# Changelog

All notable changes to this repository will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/) and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## Unreleased

### Upgrade Notes

### Known Issues

### Added

Added Sonarqube scanner

### Changed

### Deprecated

### Removed

### Fixed

Registering a topic or service that is already registered with the same type keeps the node
that is there. A Unity client that connects again registers everything it had, and each
registration used to destroy and recreate its ROS node while the executor was spinning on
another thread; now and then the executor met a node in the middle of its destruction and the
endpoint process ended (`InvalidHandle: cannot use Destroyable because destruction was
requested`). A node that does have to be replaced (another type, another queue size) is taken
out of the executor before it is destroyed.


## [0.7.1] - 2026-08-24

### Added

- Added `ros2_actions_v1` capability negotiation.
- Added concurrent ROS 2 action goals, feedback, result retrieval, cancellation,
  and explicit protocol errors for Unity action clients.
- Added a standalone Fibonacci action server and combined endpoint demo launch.

### Changed

- Action command/payload responses are queued atomically so concurrent callbacks
  cannot interleave protocol frames.


## [0.7.0] - 2022-02-01

### Added

Added Sonarqube scanner

Send information during hand shaking for ros and package version checks

Send service response as one queue item


## [0.6.0] - 2021-09-30

Add the [Close Stale Issues](https://github.com/marketplace/actions/close-stale-issues) action

### Upgrade Notes

### Known Issues

### Added

Support for queue_size and latch for publishers. (https://github.com/Unity-Technologies/ROS-TCP-Endpoint/issues/82)

### Changed

### Deprecated

### Removed

### Fixed

## [0.5.0] - 2021-07-15

### Upgrade Notes

Upgrade the ROS communication to support ROS2 with Unity

### Known Issues

### Added

### Changed

### Deprecated

### Removed

### Fixed

## [0.4.0] - 2021-05-27

Note: the logs only reflects the changes from version 0.3.0

### Upgrade Notes

RosConnection 2.0: maintain a single constant connection from Unity to the Endpoint. This is more efficient than opening one connection per message, and it eliminates a whole bunch of user issues caused by ROS being unable to connect to Unity due to firewalls, proxies, etc.

### Known Issues

### Added

Add a link to the Robotics forum, and add a config.yml to add a link in the Github Issues page

Add linter, unit tests, and test coverage reporting

### Changed

Improving the performance of the read_message in client.py, This is done by receiving the entire message all at once instead of reading 1024 byte chunks and stitching them together as you go.

### Deprecated

### Removed

Remove outdated handshake references

### Fixed
