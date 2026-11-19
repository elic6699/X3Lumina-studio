//  Copyright 2026 X3Lumina Studio contributors
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

#include "automation/handoff_cache.hpp"

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QStandardPaths>

namespace alcedo::automation {
namespace {

/// A test that writes a stale file, then checks that preparing the cache removes it.
TEST(HandoffCacheTest, PrepareCreatesDirectoryAndRemovesStaleFiles) {
  QStandardPaths::setTestModeEnabled(true);
  const QString path = HandoffCacheDirectory();
  ASSERT_FALSE(path.isEmpty());
  EXPECT_TRUE(path.endsWith(QStringLiteral("handoff-cache")));

  QDir dir(path);
  ASSERT_TRUE(dir.mkpath(path));

  const QString stale = dir.filePath(QStringLiteral("stale.tif"));
  QFile       file(stale);
  ASSERT_TRUE(file.open(QIODevice::WriteOnly));
  ASSERT_EQ(file.write("stale"), 5);
  file.close();
  ASSERT_TRUE(QFile::exists(stale));

  QString error;
  ASSERT_TRUE(PrepareHandoffCacheDirectory(&error)) << error.toStdString();
  EXPECT_TRUE(QDir(path).exists());
  EXPECT_FALSE(QFile::exists(stale));
}

}  // namespace
}  // namespace alcedo::automation
