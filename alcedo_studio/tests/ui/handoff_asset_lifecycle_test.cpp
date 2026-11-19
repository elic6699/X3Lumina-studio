//  Copyright 2026 Yurun Zi
//  SPDX-License-Identifier: GPL-3.0-only
//  Additional permission under GPLv3 section 7 applies; see the LICENSE file.

/// @file handoff_asset_lifecycle_test.cpp
/// @brief Lifecycle of temporary assets imported from the x3lumina handoff cache.
///
/// These tests drive the two signals that the handoff wiring connects in
/// ApplicationModuleHost, without needing a GPU editor backend or real RAW pixels:
///
///   * ImportLogReady registers only assets whose source file is inside the Studio-managed
///     handoff cache directory; a normal import of an unrelated file is never tracked.
///   * Shutdown removes the cache file of every asset that is still tracked (an editor that
///     never reached a normal close), while leaving unrelated files alone.
///
/// This executable is standalone because `QStandardPaths::setTestModeEnabled(true)` is a
/// process-wide, irreversible switch; it must not affect the shared UI test process.

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QStandardPaths>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

#include "automation/handoff_cache.hpp"
#include "ui/album_backend_test_fixture.hpp"
#include "ui/alcedo_main/album_backend/application_module_host.hpp"
#include "ui/alcedo_main/album_backend/import_export.hpp"
#include "utils/import/import_log.hpp"

namespace alcedo::ui::test {
namespace {

namespace fs = std::filesystem;

class HandoffAssetLifecycleTest : public ApplicationModuleHostTestFixture {
 protected:
  void SetUp() override {
    ApplicationModuleHostTestFixture::SetUp();
    QStandardPaths::setTestModeEnabled(true);
    cache_dir_ = automation::HandoffCacheDirectory();
    ASSERT_FALSE(cache_dir_.isEmpty());
    ASSERT_TRUE(QDir().mkpath(cache_dir_));
  }

  void TearDown() override {
    QDir(cache_dir_).removeRecursively();
    ApplicationModuleHostTestFixture::TearDown();
  }

  /// A cache file with unique content, so an accidental removal of an unrelated file is
  /// detectable by its continued existence.
  auto WriteCacheFile(const QString& name, const std::string& content) -> QString {
    const QString path = QDir(cache_dir_).filePath(name);
    QFile         file(path);
    EXPECT_TRUE(file.open(QIODevice::WriteOnly));
    EXPECT_EQ(file.write(content.c_str()), static_cast<qint64>(content.size()));
    file.close();
    return path;
  }

  /// Publishes an import snapshot through the same signal that CompleteImport emits. The
  /// snapshot carries the ids and the source path; no real import is needed to drive the
  /// registration handler.
  void PublishImport(const fs::path& source, image_id_t image_id, sl_element_id_t element_id) {
    auto snapshot             = std::make_shared<ImportLogSnapshot>();
    ImportLogEntry entry;
    entry.image_id_    = image_id;
    entry.element_id_  = element_id;
    entry.file_name_   = source.filename().wstring();
    entry.source_path_ = source;
    entry.metadata_ok_ = true;
    snapshot->created_.push_back(entry);
    snapshot->metadata_ok_.push_back(image_id);
    emit backend_->import_export()->ImportLogReady(snapshot);
    ProcessEvents(200);
  }

  std::unique_ptr<ApplicationModuleHost> backend_;
  QString                                cache_dir_;
};

TEST_F(HandoffAssetLifecycleTest, ShutdownRemovesTrackedCacheFilesButNotUnrelatedFiles) {
  backend_ = std::make_unique<ApplicationModuleHost>();
  ASSERT_TRUE(CreateTestProject(*backend_));

  const QString handoff = WriteCacheFile(QStringLiteral("handoff-abc123.tif"), "handoff bytes");
  const QString other   = WriteCacheFile(QStringLiteral("unrelated.tif"), "keep me");
  ASSERT_TRUE(QFile::exists(handoff));
  ASSERT_TRUE(QFile::exists(other));

  PublishImport(fs::path(handoff.toStdWString()), 41, 7);

  backend_->Shutdown();
  backend_.reset();

  EXPECT_FALSE(QFile::exists(handoff)) << "a tracked handoff cache file must be discarded";
  EXPECT_TRUE(QFile::exists(other)) << "an unrelated file must not be removed";
}

TEST_F(HandoffAssetLifecycleTest, AssetsOutsideTheCacheDirectoryAreNeverTracked) {
  backend_ = std::make_unique<ApplicationModuleHost>();
  ASSERT_TRUE(CreateTestProject(*backend_));

  // A normal import from the user's own directory whose file happens to exist. Publishing it
  // must not register it, so Shutdown must leave the file untouched.
  const QString outside =
      QDir(QString::fromStdString(temp_dir_.string())).filePath(QStringLiteral("my_photo.tif"));
  QFile         file(outside);
  ASSERT_TRUE(file.open(QIODevice::WriteOnly));
  file.write("user photo");
  file.close();
  ASSERT_TRUE(QFile::exists(outside));

  PublishImport(fs::path(outside.toStdWString()), 43, 9);

  backend_->Shutdown();
  backend_.reset();

  EXPECT_TRUE(QFile::exists(outside)) << "a normal import must never be treated as temporary";
}

}  // namespace
}  // namespace alcedo::ui::test
