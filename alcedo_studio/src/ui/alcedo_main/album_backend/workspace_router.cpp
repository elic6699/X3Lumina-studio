//  Copyright 2026 Yurun Zi
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "ui/alcedo_main/album_backend/workspace_router.hpp"

#include <QDebug>

#include "app/external_editor_bridge.hpp"
#include "ui/alcedo_main/album_backend/editor_session_controller.hpp"
#include "utils/diagnostics/app_logging.hpp"

namespace alcedo::ui {

WorkspaceRouter::WorkspaceRouter(EditorSessionController* editor_session, QObject* parent)
    : QObject(parent), editor_session_(editor_session) {}

void WorkspaceRouter::SetInteractionPolicy(InteractionPolicyController* interaction_policy) {
  interaction_policy_ = interaction_policy;
}

bool WorkspaceRouter::openEditor(uint elementId, uint imageId) {
  if (const QString refusal = UserStateChangeRefusal(interaction_policy_); !refusal.isEmpty()) {
    const bool open_image = editor_session_ != nullptr && editor_session_->has_image() &&
                            editor_session_->element_id() == elementId &&
                            editor_session_->image_id() == imageId;
    if (!open_image) {
      qCInfo(alcedo::diag::editorLog).noquote() << "workspace.openEditor refused:" << refusal;
      return false;
    }
  }
  OpenEditor(elementId, imageId);
  return true;
}

void WorkspaceRouter::OpenLibrary() {
  if (workspace_ == QStringLiteral("library") && element_id_ == 0 && image_id_ == 0) {
    return;
  }
  if (editor_session_) {
    // Parent visibility changes do not reliably notify the retained viewport.
    // Pause producer handshakes before hiding its workspace.
    editor_session_->SetWorkspacePresentationActive(false);
    // Leaving the editor closes an open comparison; the hidden viewport needs no refresh.
    editor_session_->CloseComparison(false);
    editor_session_->PersistCurrentImage();
  }
  workspace_  = QStringLiteral("library");
  element_id_ = 0;
  image_id_   = 0;
  emit RouteChanged();
}

void WorkspaceRouter::OpenEditor(uint elementId, uint imageId) {
  // Thin bridge: X3F/X3I editing is owned by the companion app. Resolve the source from the
  // image id and hand it off. When the launcher accepts, no editor workspace is opened, so the
  // built-in Develop DAG never sees a Foveon source. When the companion app is not configured
  // the launcher declines; the ordinary editor route then runs the product's existing X3F
  // path (the wide-gamut Foveon chain, not the display-referred proxy), so the photo stays
  // editable. That is configuration-not-enabled, not a degraded substitute for a failed path.
  qWarning().noquote() << "[X3LuminaBridge] OpenEditor entered:"
                       << "elementId=" << elementId << "imageId=" << imageId
                       << "resolver=" << bool(path_resolver_)
                       << "launcher=" << bool(editor_launcher_);
  if (path_resolver_ && editor_launcher_ && imageId != 0) {
    const auto source = path_resolver_(imageId);
    qWarning().noquote() << "[X3LuminaBridge] resolved source:"
                         << QString::fromStdWString(source.wstring())
                         << "managed=" << external_editor::IsExternallyManaged(source);
    if (external_editor::IsExternallyManaged(source)) {
      if (editor_launcher_(source)) {
        qCInfo(alcedo::diag::editorLog).noquote()
            << "workspace.openEditor delegated to companion app:"
            << QString::fromStdWString(source.wstring());
        return;
      }
      qCInfo(alcedo::diag::editorLog).noquote()
          << "workspace.openEditor companion app not configured; using built-in editor for:"
          << QString::fromStdWString(source.wstring());
    }
  }
  OpenEditorWithProxy(elementId, imageId);
}

void WorkspaceRouter::OpenEditorWithProxy(uint elementId, uint imageId) {
  workspace_  = QStringLiteral("editor");
  element_id_ = elementId;
  image_id_   = imageId;
  // Publish the visual route first. The viewport must be visible before Open
  // can produce a frame that requires a presentation acknowledgement.
  emit RouteChanged();
  if (editor_session_) {
    editor_session_->SetWorkspacePresentationActive(true);
    // EditorSessionController chooses Open, Switch, or Close. Do not pre-close
    // here: A→B must remain one service-owned switch operation.
    editor_session_->Open(elementId, imageId);
  }
}

}  // namespace alcedo::ui
