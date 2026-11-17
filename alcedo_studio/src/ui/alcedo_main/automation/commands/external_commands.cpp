//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "ui/alcedo_main/automation/automation_external_commands.hpp"

#include <QJsonArray>
#include <QJsonObject>
#include <QVariantMap>

#include <utility>

#include "app/thumbnail_service.hpp"
#include "sleeve/album_query.hpp"
#include "ui/alcedo_main/automation/automation_command_support.hpp"

namespace alcedo::automation {
namespace {

auto ElementIdsParam() -> QJsonObject { return AutomationElementIdListSchema(); }

void RegisterOrFail(AutomationCommandRegistry& registry, AutomationCommandSpec spec, bool* ok,
                    QString* error) {
  if (*ok && !registry.Register(std::move(spec), error)) {
    *ok = false;
  }
}

}  // namespace

auto RegisterAutomationExternalCommands(AutomationCommandRegistry& registry,
                                        ui::ApplicationModuleHost* host, QString* error) -> bool {
  bool ok = true;

  AutomationCommandSpec refresh_thumbnail;
  refresh_thumbnail.method = QStringLiteral("external.refreshThumbnail");
  refresh_thumbnail.description = QStringLiteral(
      "Drops the cached previews of 'element_ids' after a companion app wrote new sidecar "
      "JPEGs. The library renders the new previews on the next request. Only display data "
      "is invalidated: the source files, Develop documents, and export pipeline are untouched.");
  refresh_thumbnail.changes_state = false;
  refresh_thumbnail.params_schema = AutomationClosedParamsSchema(
      QJsonObject{{"element_ids", ElementIdsParam()}}, QJsonArray{"element_ids"});
  refresh_thumbnail.result_schema = AutomationObjectSchema();
  refresh_thumbnail.handler = [host](const QJsonObject& params, AutomationReply reply) {
    const auto project = AutomationEnteredProject(host, reply);
    if (!project) {
      return;
    }
    const auto rows = ResolveAutomationElementRows(*project, params, reply);
    if (!rows.has_value()) {
      return;
    }
    if (const auto thumbnails = host->project()->handler().thumbnail_service()) {
      for (const auto& row : *rows) {
        thumbnails->InvalidateThumbnail(row.file_id_);
      }
    }
    reply.SendResult(QJsonObject{{"refreshed_count", static_cast<int>(rows->size())}});
  };
  RegisterOrFail(registry, std::move(refresh_thumbnail), &ok, error);

  return ok;
}

}  // namespace alcedo::automation
