//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "automation/handoff_cache.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <QString>

namespace alcedo::automation {
namespace {

const auto kHandoffCacheDirectoryName = QStringLiteral("handoff-cache");

}  // namespace

auto HandoffCacheDirectory() -> QString {
  const QString data_dir =
      QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation);
  return QDir(data_dir).filePath(kHandoffCacheDirectoryName);
}

auto PrepareHandoffCacheDirectory(QString* error) -> bool {
  const QString path = HandoffCacheDirectory();
  QDir       dir(path);
  if (!dir.exists() && !dir.mkpath(path)) {
    if (error != nullptr) {
      *error = QStringLiteral("cannot create handoff cache directory '%1'").arg(path);
    }
    return false;
  }

  const QFileInfoList entries =
      dir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot, QDir::Name);
  for (const QFileInfo& entry : entries) {
    if (!QFile::remove(entry.absoluteFilePath())) {
      if (error != nullptr) {
        *error = QStringLiteral("cannot remove stale handoff file '%1'")
                     .arg(entry.absoluteFilePath());
      }
      return false;
    }
  }
  return true;
}

}  // namespace alcedo::automation
