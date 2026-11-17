//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <QString>

#include "ui/alcedo_main/album_backend/application_module_host.hpp"
#include "ui/alcedo_main/automation/automation_command_registry.hpp"

namespace alcedo::automation {

/// Registers `external.refreshThumbnail`. A companion app calls it over the local socket
/// after it writes a sidecar JPEG, so the library drops the cached preview of the named
/// photos and renders the new one. It only invalidates display data: no Develop document,
/// history, source file, or export pipeline is touched. @p host must outlive every dispatch.
/// Returns false and writes @p error when registration fails.
[[nodiscard]] auto RegisterAutomationExternalCommands(AutomationCommandRegistry& registry,
                                                      ui::ApplicationModuleHost* host,
                                                      QString* error = nullptr) -> bool;

}  // namespace alcedo::automation
