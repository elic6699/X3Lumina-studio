//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <QImage>
#include <QMutex>
#include <QObject>
#include <QQuickImageProvider>
#include <QString>
#include <QThreadPool>
#include <QUrl>
#include <QVariant>

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "decoders/x3f/x3f_foveon.hpp"

namespace alcedo::ui {

/// Thread-safe mailbox between the CPU render worker and QML's image provider.
/// The provider lives on the QML engine; the controller owns the renderer.
class FoveonPreviewStore final {
 public:
  void Set(QImage image, std::uint64_t revision);
  [[nodiscard]] auto Get(std::uint64_t* revision = nullptr) const -> QImage;

 private:
  mutable QMutex   mutex_;
  QImage           image_;
  std::uint64_t    revision_ = 0;
};

[[nodiscard]] auto SharedFoveonPreviewStore()
    -> std::shared_ptr<FoveonPreviewStore>;

class FoveonPreviewImageProvider final : public QQuickImageProvider {
 public:
  explicit FoveonPreviewImageProvider(std::shared_ptr<FoveonPreviewStore> store);
  [[nodiscard]] auto requestImage(const QString& id, QSize* size,
                                  const QSize& requestedSize) -> QImage override;

 private:
  std::shared_ptr<FoveonPreviewStore> store_;
};

/// QML-facing owner of path B (the Foveon native chain).
///
/// Path A (the default wide-gamut DirectRgb Develop DAG) is completely
/// untouched: enabling this controller only overlays an independent,
/// display-referred 16-bit sRGB preview computed on a CPU worker, and writes a
/// separate .x3fedit.json sidecar. No DevelopPayload / catalog / history /
/// content-key state is involved.
class FoveonSessionController final : public QObject {
  Q_OBJECT
  Q_PROPERTY(bool available READ available NOTIFY SourceChanged)
  Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY StateChanged)
  Q_PROPERTY(QString sourcePath READ sourcePath NOTIFY SourceChanged)
  Q_PROPERTY(QString parametersJson READ parametersJson NOTIFY ParametersChanged)
  Q_PROPERTY(QString valuesJson READ valuesJson NOTIFY ParametersChanged)
  Q_PROPERTY(int previewRevision READ previewRevision NOTIFY PreviewChanged)
  Q_PROPERTY(QString status READ status NOTIFY StateChanged)
  Q_PROPERTY(int modelClass READ modelClass NOTIFY SourceChanged)
  /// Gate generation (SDQ / SDQH count as Quattro). Panel gating uses this.
  Q_PROPERTY(int modelGen READ modelGen NOTIFY SourceChanged)
  /// True when "SDQ 高光" can affect this file (DigitalISOGain != 1).
  Q_PROPERTY(bool sdqHlCapable READ sdqHlCapable NOTIFY SourceChanged)
  Q_PROPERTY(int availableDecoders READ availableDecoders NOTIFY SourceChanged)
  Q_PROPERTY(bool sppCapable READ sppCapable NOTIFY SourceChanged)
  /// Global Merrill decode backend: "fuse" (default) or "spp" (原链解码).
  /// X3Lumina keeps this in a global preference, NOT in the per-file sidecar.
  Q_PROPERTY(QString decoderBackend READ decoderBackend NOTIFY DecoderChanged)
  /// Only Merrill files with a usable SPP chain offer the two-state choice.
  Q_PROPERTY(bool decoderChoiceAvailable READ decoderChoiceAvailable NOTIFY SourceChanged)
  /// True when this image actually renders through the SPP native chain.
  Q_PROPERTY(bool sppMode READ sppMode NOTIFY DecoderChanged)
  /// True while an independent 16-bit sRGB export is running on the worker.
  Q_PROPERTY(bool exporting READ exporting NOTIFY ExportStateChanged)
  /// Digest of the Foveon decode parameters currently live in the decoder
  /// bridge. The panel echoes it back through the RAW Decode node so a slider
  /// move is a document change (re-render, history, result-key) rather than a
  /// silent sidecar edit. 0 means the native operators are not applied.
  Q_PROPERTY(QString decodeDigest READ decodeDigest NOTIFY DecodeParamsChanged)

 public:
  /// Resolves the on-disk source path of the currently open image id. Injected
  /// by ApplicationModuleHost so this controller does not depend on the project
  /// / image-pool service internals.
  using PathResolver = std::function<std::filesystem::path(std::uint32_t)>;

  explicit FoveonSessionController(PathResolver resolver,
                                   QObject* parent = nullptr);
  ~FoveonSessionController() override;

  [[nodiscard]] auto available() const -> bool { return available_; }
  [[nodiscard]] auto enabled() const -> bool { return enabled_; }
  [[nodiscard]] auto sourcePath() const -> QString { return source_path_; }
  [[nodiscard]] auto status() const -> QString { return status_; }
  [[nodiscard]] auto modelClass() const -> int { return caps_ ? caps_->model_class : -1; }
  [[nodiscard]] auto modelGen() const -> int { return caps_ ? caps_->model_gen : -1; }
  [[nodiscard]] auto sdqHlCapable() const -> bool {
    return caps_ && caps_->sdq_hl_capable;
  }
  [[nodiscard]] auto availableDecoders() const -> int {
    return caps_ ? caps_->available_decoders : 0;
  }
  [[nodiscard]] auto sppCapable() const -> bool { return caps_ && caps_->spp_capable; }
  [[nodiscard]] auto previewRevision() const -> int {
    return static_cast<int>(revision_);
  }
  [[nodiscard]] auto decoderBackend() const -> QString { return decoder_backend_; }
  [[nodiscard]] auto decoderChoiceAvailable() const -> bool {
    return decoder_choice_available_;
  }
  [[nodiscard]] auto sppMode() const -> bool {
    return decoder_choice_available_ &&
           decoder_backend_ == QStringLiteral("spp");
  }
  [[nodiscard]] auto parametersJson() const -> QString;
  [[nodiscard]] auto valuesJson() const -> QString;
  [[nodiscard]] auto exporting() const -> bool { return exporting_; }
  /// 64-bit digest as text: QML numbers are doubles and would lose the low
  /// bits of a 64-bit hash, which is exactly what identifies a decode.
  [[nodiscard]] auto decodeDigest() const -> QString {
    return QString::number(static_cast<qulonglong>(digest_));
  }
  /// Worker-side check: a queued render whose generation is stale can return
  /// before paying the SPP prepare cost.
  [[nodiscard]] auto IsCurrentRenderGeneration(std::uint64_t generation) const
      -> bool {
    return generation == generation_.load();
  }

  /// URL for the QML overlay; the revision busts the QML image cache.
  Q_INVOKABLE [[nodiscard]] QString previewUrl() const;

  /// Independent 16-bit sRGB export of the active path-B / SPP product.
  /// Requires path B to be enabled or Merrill SPP mode to be selected; it
  /// never writes through the wide-gamut path-A export pipeline.
  Q_INVOKABLE void ExportFoveon16(const QUrl& destination);

 public slots:
  void SyncFromImageId(std::uint32_t image_id);
  void SetSourcePath(QString path);
  void setEnabled(bool on);
  void SetParam(QString key, QVariant value);
  void ResetParams();
  void LoadSidecar();
  void SaveSidecar();
  void RequestPreview();
  void SetDecoderBackend(const QString& backend);

  /// Invoked on the GUI thread by the render worker. Public so a QRunnable can
  /// deliver the result through a queued invocation.
  void PublishPreview(std::uint64_t generation, QImage image, bool ok,
                      QString message);
  void PublishExport(bool ok, QString message);

 signals:
  void SourceChanged();
  void StateChanged();
  void ParametersChanged();
  void PreviewChanged();
  void DecoderChanged();
  void DecodeParamsChanged();
  void ExportStateChanged();
  void ExportFinished(bool ok, QString message);

 private:
  void ScheduleRender();
  /// Publish (or withdraw) this file's Foveon operators in the decoder bridge.
  /// Publish is keyed by the encoded content hash, so the render worker's
  /// decode and PreparedSourceCache agree on identity.
  void PublishDecodeParams();

  PathResolver                                resolver_;
  alcedo::x3f::FoveonEditParams               params_;
  std::optional<alcedo::x3f::FoveonCapabilities> caps_;
  std::shared_ptr<alcedo::x3f::FoveonRenderer> renderer_;
  std::shared_ptr<FoveonPreviewStore>          store_;
  QThreadPool                                 pool_;
  QString                                     source_path_;
  QString                                     status_;
  QString                                     decoder_backend_ = QStringLiteral("fuse");
  bool                                        decoder_choice_available_ = false;
  bool                                        available_ = false;
  /// The Foveon native chain is an explicit sidecar, not the default view:
  /// X3F now opens on path A (own for Quattro, fuse for classic, with the
  /// Merrill SPP choice available), which is the colour-corrected pipeline.
  /// The flag stays so the native-chain-only controls have a carrier until
  /// they are folded into the Develop graph.
  bool                                        path_b_preferred_ = false;
  bool                                        enabled_   = false;
  bool                                        exporting_ = false;
  std::atomic<std::uint64_t>                  generation_{0};
  std::uint64_t                               revision_  = 0;
  /// Encoded content hash of the open X3F (the key the decode cache uses).
  std::uint64_t                               content_hash_ = 0;
  /// Digest last published to the bridge; echoed to QML as decodeDigest.
  std::uint64_t                               digest_       = 0;
  int                                         preview_long_edge_ = 1280;
};

}  // namespace alcedo::ui
