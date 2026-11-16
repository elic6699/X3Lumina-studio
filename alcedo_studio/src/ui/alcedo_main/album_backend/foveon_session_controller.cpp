//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "ui/alcedo_main/album_backend/foveon_session_controller.hpp"

#include <QMetaObject>
#include <QMetaType>
#include <QMutexLocker>
#include <QRunnable>
#include <QSettings>
#include <QTransform>

#include <algorithm>
#include <cctype>
#include <utility>

#include "app/foveon_edit_params_io.hpp"
#include "decoders/x3f/x3f_support.hpp"
#include "ui/alcedo_main/album_backend/foveon_export_writer.hpp"

namespace alcedo::ui {
namespace {

using alcedo::x3f::FoveonCapabilities;
using alcedo::x3f::FoveonEditParams;
using alcedo::x3f::FoveonImage;
using alcedo::x3f::FoveonRenderer;

auto SharedStore() -> std::shared_ptr<FoveonPreviewStore> {
  static std::shared_ptr<FoveonPreviewStore> store =
      std::make_shared<FoveonPreviewStore>();
  return store;
}

/// 16-bit sRGB display code values -> 8-bit preview. The values are already
/// display-encoded: taking the high byte is correct, no gamma is applied.
auto ToQImage(const FoveonImage& source) -> QImage {
  QImage image(source.width, source.height, QImage::Format_RGB888);
  if (image.isNull()) {
    return {};
  }
  const auto* src = source.rgb.get();
  for (int y = 0; y < source.height; ++y) {
    auto* row = image.scanLine(y);
    const auto* in =
        src + static_cast<std::size_t>(y) * source.width * 3U;
    for (int x = 0; x < source.width; ++x) {
      row[x * 3 + 0] = static_cast<uchar>(in[x * 3 + 0] >> 8);
      row[x * 3 + 1] = static_cast<uchar>(in[x * 3 + 1] >> 8);
      row[x * 3 + 2] = static_cast<uchar>(in[x * 3 + 2] >> 8);
    }
  }
  // The C frame is in sensor order; apply the EXIF orientation exactly once.
  switch (source.orientation) {
    case 2: return image.mirrored(true, false);
    case 3: return image.transformed(QTransform().rotate(180));
    case 4: return image.mirrored(false, true);
    case 5: return image.transformed(QTransform().rotate(90)).mirrored(true, false);
    case 6: return image.transformed(QTransform().rotate(90));
    case 7: return image.transformed(QTransform().rotate(270)).mirrored(true, false);
    case 8: return image.transformed(QTransform().rotate(270));
    default: return image;
  }
}

class FoveonRenderTask final : public QRunnable {
 public:
  FoveonRenderTask(FoveonSessionController* owner,
                   std::shared_ptr<FoveonRenderer> renderer,
                   std::filesystem::path path, FoveonEditParams params,
                   std::optional<FoveonCapabilities> caps,
                   std::uint64_t generation, int long_edge, bool spp_mode)
      : owner_(owner),
        renderer_(std::move(renderer)),
        path_(std::move(path)),
        params_(std::move(params)),
        caps_(std::move(caps)),
        generation_(generation),
        long_edge_(long_edge),
        spp_mode_(spp_mode) {
    setAutoDelete(true);
  }

  void run() override {
    if (!owner_->IsCurrentRenderGeneration(generation_)) {
      return;  // a newer slider change superseded this queued render
    }
    std::optional<FoveonImage> image;
    QString message;
    if (spp_mode_) {
      // Merrill 原链解码: the SPP chain renders a full-resolution 16-bit sRGB
      // finished image. The six SPP tone controls are applied inside SPP's
      // own tone stage; the renderer caches prepare() and only re-runs tone
      // on look-slider changes.
      image = renderer_->RenderSpp(path_, params_, -1);
      if (!image) {
        message = QString::fromStdString(renderer_->LastError());
        if (message.isEmpty()) {
          message = QStringLiteral("SPP native-chain render failed "
                                   "(Merrill HI-resolution file required)");
        }
      }
    } else {
      // Preview resolution is derived from the long edge; the sidecar's `down`
      // stays available for full-resolution one-shot renders/export.
      if (caps_) {
        params_.down = alcedo::x3f::FoveonPreviewDown(*caps_, long_edge_);
      }
      image = renderer_->Render(path_, params_);
      if (!image) {
        message = QString::fromStdString(renderer_->LastError());
        if (message.isEmpty()) {
          message = QString::fromStdString(alcedo::x3f::FoveonLastError());
        }
      }
    }
    QImage preview;
    if (image && !image->Empty()) {
      preview = ToQImage(*image);
      if (spp_mode_ &&
          std::max(preview.width(), preview.height()) > long_edge_) {
        preview = preview.scaled(long_edge_, long_edge_, Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);
      }
    } else if (message.isEmpty()) {
      message = QStringLiteral("Foveon native-chain render failed");
    }
    const bool ok = !preview.isNull();
    // The queued invocation is dropped automatically if the controller has
    // been destroyed in the meantime.
    QMetaObject::invokeMethod(
        owner_,
        [owner = owner_, gen = generation_, preview = std::move(preview), ok,
         message = std::move(message)]() mutable {
          owner->PublishPreview(gen, std::move(preview), ok, std::move(message));
        },
        Qt::QueuedConnection);
  }

 private:
  FoveonSessionController*           owner_;
  std::shared_ptr<FoveonRenderer>    renderer_;
  std::filesystem::path              path_;
  FoveonEditParams                   params_;
  std::optional<FoveonCapabilities>  caps_;
  std::uint64_t                      generation_;
  int                                long_edge_;
  bool                               spp_mode_;
};

/// Full-resolution one-shot export for the independent 16-bit sRGB product.
/// It shares the single worker with preview rendering, but it is not part of
/// the wide-gamut path-A export queue and never mutates path-A state.
class FoveonExportTask final : public QRunnable {
 public:
  FoveonExportTask(FoveonSessionController* owner,
                   std::shared_ptr<FoveonRenderer> renderer,
                   std::filesystem::path path, FoveonEditParams params,
                   std::filesystem::path destination, bool spp_mode)
      : owner_(owner),
        renderer_(std::move(renderer)),
        path_(std::move(path)),
        params_(std::move(params)),
        destination_(std::move(destination)),
        spp_mode_(spp_mode) {
    setAutoDelete(true);
  }

  void run() override {
    std::optional<FoveonImage> image;
    QString message;
    if (spp_mode_) {
      image = renderer_->RenderSpp(path_, params_, -1);
      if (!image) {
        message = QString::fromStdString(renderer_->LastError());
      }
    } else {
      // Export always renders the full-resolution path-B product, never the
      // preview down-sample.
      params_.down = 1;
      image = renderer_->Render(path_, params_);
      if (!image) {
        message = QString::fromStdString(renderer_->LastError());
      }
    }

    bool ok = false;
    if (image && !image->Empty()) {
      std::string error;
      ok = WriteFoveon16Bit(*image, destination_, &error);
      if (ok) {
        message = QStringLiteral("16-bit sRGB exported: %1")
                      .arg(QString::fromStdString(destination_.string()));
      } else {
        message = QString::fromStdString(error);
        if (message.isEmpty()) {
          message = QStringLiteral("16-bit sRGB export failed");
        }
      }
    } else if (message.isEmpty()) {
      message = QStringLiteral("16-bit sRGB export: no image");
    }

    QMetaObject::invokeMethod(
        owner_,
        [owner = owner_, ok, message = std::move(message)]() mutable {
          owner->PublishExport(ok, std::move(message));
        },
        Qt::QueuedConnection);
  }

 private:
  FoveonSessionController*        owner_;
  std::shared_ptr<FoveonRenderer> renderer_;
  std::filesystem::path           path_;
  FoveonEditParams                params_;
  std::filesystem::path           destination_;
  bool                            spp_mode_;
};

}  // namespace

void FoveonPreviewStore::Set(QImage image, std::uint64_t revision) {
  const QMutexLocker lock(&mutex_);
  image_    = std::move(image);
  revision_ = revision;
}

auto FoveonPreviewStore::Get(std::uint64_t* revision) const -> QImage {
  const QMutexLocker lock(&mutex_);
  if (revision != nullptr) {
    *revision = revision_;
  }
  return image_;
}

auto SharedFoveonPreviewStore() -> std::shared_ptr<FoveonPreviewStore> {
  return SharedStore();
}

FoveonPreviewImageProvider::FoveonPreviewImageProvider(
    std::shared_ptr<FoveonPreviewStore> store)
    : QQuickImageProvider(QQuickImageProvider::Image),
      store_(std::move(store)) {}

auto FoveonPreviewImageProvider::requestImage(const QString& id, QSize* size,
                                              const QSize& requestedSize)
    -> QImage {
  Q_UNUSED(id);
  Q_UNUSED(requestedSize);
  auto image = store_ ? store_->Get() : QImage();
  if (size != nullptr) {
    *size = image.size();
  }
  return image;
}

FoveonSessionController::FoveonSessionController(PathResolver resolver,
                                                 QObject* parent)
    : QObject(parent),
      resolver_(std::move(resolver)),
      renderer_(std::make_shared<FoveonRenderer>()),
      store_(SharedStore()) {
  // The legacy C chain uses process-global switches; one worker keeps the
  // prepare/apply cache coherent and prevents concurrent mode swaps.
  pool_.setMaxThreadCount(1);
  // Global preference (X3Lumina stores this in UserDefaults, never in the
  // per-file sidecar). Accept both our short names and X3Lumina's raw values.
  const QSettings settings;
  const QString saved =
      settings.value(QStringLiteral("x3f.merrillDecodeBackend"),
                     QStringLiteral("x3fuse"))
          .toString();
  decoder_backend_ =
      (saved == QStringLiteral("spp") || saved == QStringLiteral("sppPipeline"))
          ? QStringLiteral("spp")
          : QStringLiteral("fuse");
  // The native chain is opt-in: X3F opens on path A, whose colour is now the
  // calibrated one on every body. The flag survives file switches so the
  // native-chain-only controls keep their carrier.
  path_b_preferred_ =
      settings.value(QStringLiteral("x3f.foveonPathB"), false).toBool();
  enabled_ = path_b_preferred_;
  status_ = QStringLiteral("Foveon native chain idle");
}

FoveonSessionController::~FoveonSessionController() {
  pool_.waitForDone();
}

auto FoveonSessionController::parametersJson() const -> QString {
  return QString::fromStdString(
      alcedo::x3f::FoveonParametersJson(params_).dump());
}

auto FoveonSessionController::valuesJson() const -> QString {
  return QString::fromStdString(alcedo::x3f::FoveonValuesJson(params_).dump());
}

auto FoveonSessionController::previewUrl() const -> QString {
  return QStringLiteral("image://foveon_preview/%1").arg(revision_);
}

void FoveonSessionController::SyncFromImageId(std::uint32_t image_id) {
  if (!resolver_) {
    return;
  }
  const auto path = resolver_(image_id);
  SetSourcePath(QString::fromStdString(path.string()));
}

void FoveonSessionController::SetSourcePath(QString path) {
  const QString normalized = std::move(path);
  if (normalized == source_path_) {
    return;
  }
  // Any queued render for the previous source is stale.
  generation_.fetch_add(1);
  const bool had_visible_preview = enabled_ || sppMode();
  source_path_ = normalized;
  const std::filesystem::path native(normalized.toStdString());
  std::error_code ec;
  if (normalized.isEmpty() || !alcedo::x3f::IsX3fFile(native) ||
      !std::filesystem::exists(native, ec)) {
    caps_.reset();
    available_ = false;
    decoder_choice_available_ = false;
    status_ = QStringLiteral("No X3F/X3I source");
    if (had_visible_preview) {
      store_->Set(QImage(), revision_);
      emit PreviewChanged();
    }
    emit SourceChanged();
    emit StateChanged();
    return;
  }

  caps_      = alcedo::x3f::FoveonProbe(native);
  available_ = caps_.has_value();
  // Quattro is fixed to own, classic to fuse (AUTO in path A). Only Merrill
  // files with a usable SPP chain get the two-state backend picker.
  decoder_choice_available_ = caps_ && caps_->OffersDecoderChoice();
  params_    = FoveonEditParams{};
  const bool loaded = alcedo::x3f::LoadFoveonSidecar(native, &params_);
  status_ = available_
                ? (loaded ? QStringLiteral("Sidecar loaded")
                          : QStringLiteral("Product defaults"))
                : QStringLiteral("Foveon probe failed");
  renderer_->Invalidate();
  // Path B owns the picture by default, so opening a file must not require a
  // manual toggle. The preference survives file switches.
  enabled_ = path_b_preferred_;
  emit SourceChanged();
  emit ParametersChanged();
  emit StateChanged();
  if (enabled_ || sppMode()) {
    ScheduleRender();
  }
}

void FoveonSessionController::SetDecoderBackend(const QString& backend) {
  const QString normalized =
      (backend == QStringLiteral("spp") ||
       backend == QStringLiteral("sppPipeline") ||
       backend == QStringLiteral("原链解码"))
          ? QStringLiteral("spp")
          : QStringLiteral("fuse");
  if (decoder_backend_ == normalized) {
    return;
  }
  // Both chains have different products; drop any queued render from the
  // previous backend before scheduling the new one.
  generation_.fetch_add(1);
  decoder_backend_ = normalized;
  QSettings settings;
  settings.setValue(QStringLiteral("x3f.merrillDecodeBackend"),
                    normalized == QStringLiteral("spp")
                        ? QStringLiteral("sppPipeline")
                        : QStringLiteral("x3fuse"));
  // The two chains produce different products; every cached frame is stale.
  renderer_->Invalidate();
  emit DecoderChanged();
  if (sppMode()) {
    status_ = QStringLiteral("Rendering SPP native chain (原链解码)...");
    ScheduleRender();
  } else if (!enabled_) {
    store_->Set(QImage(), revision_);
    status_ = QStringLiteral("Wide-gamut DirectRgb path (A)");
    emit PreviewChanged();
  } else {
    ScheduleRender();
  }
  emit StateChanged();
}

void FoveonSessionController::setEnabled(bool on) {
  if (enabled_ == on) {
    return;
  }
  enabled_ = on;
  path_b_preferred_ = on;
  {
    QSettings settings;
    settings.setValue(QStringLiteral("x3f.foveonPathB"), on);
  }
  if (!on) {
    // Path A resumes as the only visible image, unless the Merrill SPP
    // backend is active: SPP直出 owns the preview independently of path B.
    if (!sppMode()) {
      store_->Set(QImage(), revision_);
      emit PreviewChanged();
    }
    status_ = sppMode() ? QStringLiteral("SPP native chain (原链解码)")
                        : QStringLiteral("Wide-gamut DirectRgb path (A)");
  } else {
    status_ = QStringLiteral("Rendering Foveon native chain (B)...");
    ScheduleRender();
  }
  emit StateChanged();
}

void FoveonSessionController::SetParam(QString key, QVariant value) {
  const std::string k = key.toStdString();
  bool changed = false;
  if (value.typeId() == QMetaType::QString) {
    changed = alcedo::x3f::FoveonSetStringParam(
        params_, k, value.toString().toStdString());
  } else {
    changed = alcedo::x3f::FoveonSetParam(params_, k, value.toDouble());
  }
  if (!changed) {
    status_ = QStringLiteral("Unknown Foveon parameter: %1").arg(key);
    emit StateChanged();
    return;
  }
  emit ParametersChanged();
  if (enabled_ || sppMode()) {
    ScheduleRender();
  }
}

void FoveonSessionController::ResetParams() {
  params_ = FoveonEditParams{};
  status_ = QStringLiteral("Product defaults (three highlight fixes ON)");
  emit ParametersChanged();
  emit StateChanged();
  if (enabled_ || sppMode()) {
    ScheduleRender();
  }
}

void FoveonSessionController::LoadSidecar() {
  if (!available_) {
    return;
  }
  if (alcedo::x3f::LoadFoveonSidecar(source_path_.toStdString(), &params_)) {
    status_ = QStringLiteral("Sidecar loaded");
  } else {
    status_ = QStringLiteral("No sidecar; kept current parameters");
  }
  emit ParametersChanged();
  emit StateChanged();
  if (enabled_ || sppMode()) {
    ScheduleRender();
  }
}

void FoveonSessionController::SaveSidecar() {
  if (!available_) {
    return;
  }
  const bool ok =
      alcedo::x3f::SaveFoveonSidecar(source_path_.toStdString(), params_);
  status_ = ok ? QStringLiteral("Sidecar saved")
               : QStringLiteral("Sidecar save failed");
  emit StateChanged();
}

void FoveonSessionController::RequestPreview() {
  if (enabled_ || sppMode()) {
    ScheduleRender();
  }
}

void FoveonSessionController::ExportFoveon16(const QUrl& destination) {
  if (exporting_) {
    return;
  }
  if (!available_ || source_path_.isEmpty()) {
    status_ = QStringLiteral("No X3F/X3I source to export");
    emit StateChanged();
    return;
  }
  if (!enabled_ && !sppMode()) {
    status_ = QStringLiteral(
        "Switch to the Foveon native chain (B) or Merrill SPP mode before "
        "exporting the 16-bit sRGB product");
    emit StateChanged();
    return;
  }

  std::filesystem::path output(destination.toLocalFile().toStdString());
  if (output.empty()) {
    status_ = QStringLiteral("No export destination selected");
    emit StateChanged();
    return;
  }
  auto extension = output.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  if (extension.empty()) {
    output += ".tif";
  } else if (extension != ".tif" && extension != ".tiff" &&
             extension != ".png") {
    status_ = QStringLiteral(
        "16-bit sRGB export supports .tif/.tiff/.png only");
    emit StateChanged();
    return;
  }

  exporting_ = true;
  status_ = sppMode() ? QStringLiteral("Exporting SPP 16-bit sRGB...")
                      : QStringLiteral("Exporting Foveon 16-bit sRGB...");
  emit ExportStateChanged();
  emit StateChanged();
  pool_.start(new FoveonExportTask(
      this, renderer_, std::filesystem::path(source_path_.toStdString()),
      params_, std::move(output), sppMode()));
}

void FoveonSessionController::ScheduleRender() {
  if ((!enabled_ && !sppMode()) || !available_ || source_path_.isEmpty()) {
    return;
  }
  const auto generation = generation_.fetch_add(1) + 1;
  status_ = sppMode() ? QStringLiteral("Rendering SPP native chain (原链解码)...")
                      : QStringLiteral("Rendering Foveon native chain (B)...");
  emit StateChanged();
  pool_.start(new FoveonRenderTask(
      this, renderer_, std::filesystem::path(source_path_.toStdString()),
      params_, caps_, generation, preview_long_edge_, sppMode()));
}

void FoveonSessionController::PublishPreview(std::uint64_t generation,
                                             QImage image, bool ok,
                                             QString message) {
  if (generation != generation_.load()) {
    return;  // a newer parameter change superseded this frame
  }
  if (!enabled_ && !sppMode()) {
    return;  // user switched back to path A while the worker was running
  }
  store_->Set(std::move(image), generation);
  ++revision_;
  status_ = ok ? (sppMode() ? QStringLiteral("SPP native chain (原链解码) ready")
                            : QStringLiteral("Foveon native chain (B) ready"))
               : std::move(message);
  emit PreviewChanged();
  emit StateChanged();
}

void FoveonSessionController::PublishExport(bool ok, QString message) {
  exporting_ = false;
  status_ = std::move(message);
  emit ExportStateChanged();
  emit StateChanged();
  emit ExportFinished(ok, status_);
}

}  // namespace alcedo::ui
