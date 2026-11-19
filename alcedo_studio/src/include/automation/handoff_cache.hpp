//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#pragma once

#include <QString>

namespace alcedo::automation {

/// Absolute path of the Studio-managed handoff cache directory. Uses the same organization
/// and application names as the automation session directory, so the companion app can find it
/// next to the session file's parent.
[[nodiscard]] auto HandoffCacheDirectory() -> QString;

/// Creates the cache directory when missing and removes every regular file in it. Called once
/// when the GUI host starts an automation session, so files left by a crashed run are gone.
/// Returns false and sets @p error when the directory cannot be prepared.
[[nodiscard]] auto PrepareHandoffCacheDirectory(QString* error = nullptr) -> bool;

}  // namespace alcedo::automation
