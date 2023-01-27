This document is a declaration of software quality for the `rclcpp_components` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

> 本文件是基于[REP-2004]中的指南的`rclcppp_components`包的软件质量声明(https://www.ros.org/reps/rep-2004.html).

# rclcpp_components Quality Declaration

The package `rclcpp_components` claims to be in the **Quality Level 1** category.

> 包`rclcpp_components`声称属于**质量级别 1**类别。

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Quality Categories in REP-2004](https://www.ros.org/reps/rep-2004.html) of the ROS2 developer guide.

> 以下是根据[REP-2004 中的包装质量类别]中列出的每个要求组织的该索赔的理由、注释和警告(https://www.ros.org/reps/rep-2004.html)ROS2开发者指南。

## Version Policy [1]

### Version Scheme [1.i]

`rclcpp_components` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#versioning).

> `rclcpp_components` 根据[ROS 2 开发者指南]中的 ROS Core 包建议使用`semver`(https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#versioning).

### Version Stability [1.ii]

`rclcpp_components` is at a stable version, i.e. `>= 1.0.0`.

> `rclcpp_components`处于稳定版本，即`>=1.0.0`。

The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

> 当前版本可以在其[package.xml]（package.xml）中找到，其更改历史可以在其[CHANGELOG]（CHANGELOG.rst）中找到。

### Public API Declaration [1.iii]

All symbols in the installed headers are considered part of the public API.

> 已安装标头中的所有符号都被视为公共 API 的一部分。

All installed headers are in the `include` directory of the package, headers in any other folders are not installed and considered private.

> 所有已安装的标头都在包的`include`目录中，任何其他文件夹中的标头都不会安装，并被视为专用。

### API Stability Policy [1.iv]

`rclcpp_components` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

> `rclcpp_components`不会破坏已发布的 ROS 发行版中的公共 API，即一旦发布了 ROS 发行版本，就不会有重大发布。

### ABI Stability Policy [1.v]

`rclcpp_components` contains C++ code and therefore must be concerned with ABI stability, and will maintain ABI stability within a ROS distribution.

> `rclcpp_components`包含 C++代码，因此必须关注 ABI 稳定性，并将在 ROS 分布中保持 ABI 稳定性。

### ABI and ABI Stability Within a Released ROS Distribution [1.vi]

`rclcpp_components` will not break API nor ABI within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

> `rclcpp_components`不会破坏已发布的 ROS 发行版中的 API 或 ABI，即一旦发布 ROS 发行版本，就不会有重大发布。

## Change Control Process [2]

`rclcpp_components` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-control-process).

> `rclcpp_components`遵循[ROS 2 开发人员指南]中建议的 ROS 核心包指南(https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-控制过程）。

### Change Requests [2.i]

All changes will occur through a pull request, check [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-control-process) for additional information.

> 所有更改都将通过拉取请求进行，请查看[ROS 2 开发者指南](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-控制过程）以获取附加信息。

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

> 此软件包使用 DCO 作为其贡献者来源确认策略。更多信息可在[CONTRIBUTING]（../CONTRIBUTING.md）中找到。

### Peer Review Policy [2.iii]

All pull requests will be peer-reviewed, check [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-control-process) for additional information.

> 所有拉取请求都将经过同行评审，请查看[ROS 2 开发者指南](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-控制过程）以获取附加信息。

### Continuous Integration [2.iv]

All pull requests must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers)

> 所有拉取请求必须在所有[第 1 层平台]上传递 CI(https://www.ros.org/reps/rep-2000.html#support-分层）

Currently nightly results can be seen here:

> 目前每晚的结果可以在这里看到：

- [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rclcpp_components/)

> -[linux-arch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rclcpp_components/)

- [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rclcpp_components/)

> -[linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rclcpp_components/)

- [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rclcpp_components/)

> -[mac*osx*发布](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rclcpp_components/)

- [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rclcpp_components/)

> -[windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rclcpp_components/)

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

> 所有拉取请求必须在合并之前解决相关文档更改。

## Documentation [3]

### Feature Documentation [3.i]

`rclcpp_components` has a [feature list](http://docs.ros2.org/latest/api/rclcpp_components/) and each item in the list links to the corresponding feature documentation. There is documentation for all of the features, and new features require documentation before being added.

> `rclcpp_components`具有[功能列表](http://docs.ros2.org/latest/api/rclcpp_components/)并且列表中的每个项目都链接到相应的功能文档。所有功能都有文档，新功能在添加之前需要文档。

### Public API Documentation [3.ii]

The API is publicly available in its [ROS 2 API documentation](http://docs.ros2.org/latest/api/rclcpp_components/).

> API 在其[ROS 2 API 文档中公开(http://docs.ros2.org/latest/api/rclcpp_components/).

### License [3.iii]

The license for `rclcpp_components` is Apache 2.0, and a summary is in each source file, the type is declared in the [`package.xml`](./package.xml) manifest file, and a full copy of the license is in the [`LICENSE`](../LICENSE) file.

> `rclcpp_components`的许可证是 Apache 2.0，每个源文件中都有摘要，类型在[`package.xml`]（./package.xml）清单文件中声明，许可证的完整副本在[`license`]（../license）文件中。

There is an automated test which runs a linter that ensures each file has a license statement. [Here](http://build.ros2.org/view/Rpr/job/Rpr__rclcpp__ubuntu_focal_amd64/lastCompletedBuild/testReport/rclcpp_components/) can be found a list with the latest results of the various linters being run on the package.

> 有一个自动测试，它运行一个 linter，确保每个文件都有一个许可证声明。[此处](http://build.ros2.org/view/Rpr/job/Rpr__rclcpp__ubuntu_focal_amd64/lastCompletedBuild/testReport/rclcpp_components/)可以找到一个包含在包上运行的各种 linter 的最新结果的列表。

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `rclcpp_components`.

> 版权所有者各自在`rclcpp_components`中的每个源代码文件中提供版权声明。

There is an automated test which runs a linter that ensures each file has at least one copyright statement. Latest linter result report can be seen [here](http://build.ros2.org/view/Rpr/job/Rpr__rclcpp__ubuntu_focal_amd64/lastCompletedBuild/testReport/rclcpp_components/copyright).

> 有一个自动测试，它运行一个 linter，确保每个文件至少有一个版权声明。可以看到最新的 linter 结果报告[此处](http://build.ros2.org/view/Rpr/job/Rpr__rclcpp__ubuntu_focal_amd64/lastCompletedBuild/testReport/rclcpp_components/copyright).

## Testing [4]

### Feature Testing [4.i]

Each feature in `rclcpp_components` has corresponding tests which simulate typical usage, and they are located in the [`test`](https://github.com/ros2/rclcpp_components/tree/master/test) directory.

> `rclcpp_components`中的每个功能都有模拟典型用法的相应测试，它们位于[`测试`]中(https://github.com/ros2/rclcpp_components/tree/master/test)目录。

New features are required to have tests before being added.

> 新功能在添加之前需要进行测试。

Currently nightly test results can be seen here:

> 当前夜间测试结果可在此处查看：

- [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rclcpp_components/)

> -[linux-arch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rclcpp_components/)

- [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rclcpp_components/)

> -[linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rclcpp_components/)

- [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rclcpp_components/)

> -[mac*osx*发布](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rclcpp_components/)

- [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rclcpp_components/)

> -[windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rclcpp_components/)

### Public API Testing [4.ii]

Each part of the public API has tests, and new additions or changes to the public API require tests before being added.

> 公共 API 的每个部分都有测试，对公共 API 的新添加或更改需要在添加之前进行测试。

The tests aim to cover both typical usage and corner cases, but are quantified by contributing to code coverage.

> 这些测试旨在涵盖典型的使用和角落情况，但通过对代码覆盖率的贡献来量化。

### Coverage [4.iii]

`rclcpp_components` follows the recommendations for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#code-coverage), and opts to use line coverage instead of branch coverage.

> `rclcpp_components`遵循[ROS 2 开发者指南]中对 ROS Core 包的建议(https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#code-覆盖），并选择使用线路覆盖而不是分支覆盖。

This includes:

> 这包括：

- tracking and reporting line coverage statistics

> -跟踪和报告线路覆盖率统计信息

- achieving and maintaining a reasonable branch line coverage (90-100%)

> -实现并保持合理的支线覆盖率（90-100%）

- no lines are manually skipped in coverage calculations

> -在覆盖率计算中不手动跳过任何行

Changes are required to make a best effort to keep or increase coverage before being accepted, but decreases are allowed if properly justified and accepted by reviewers.

> 更改需要在被接受之前尽最大努力保持或增加覆盖范围，但如果有适当的理由并被评审员接受，则允许减少覆盖范围。

Current coverage statistics can be viewed [here](https://ci.ros2.org/job/nightly_linux_coverage/lastCompletedBuild/cobertura/src_ros2_rclcpp_rclcpp_components_src/). A description of how coverage statistics are calculated is summarized in this page ["ROS 2 Onboarding Guide"](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#note-on-coverage-runs).

> 当前覆盖率统计数据可查看[此处](https://ci.ros2.org/job/nightly_linux_coverage/lastCompletedBuild/cobertura/src_ros2_rclcpp_rclcpp_components_src/). 本页总结了如何计算覆盖率统计数据[`ROS 2 入职指南`](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#note-在覆盖运行时）。

### Performance [4.iv]

`rclcpp_components` follows the recommendations for performance testing of C/C++ code in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#performance), and opts to do performance analysis on each release rather than each change.

> `rclcpp_components 遵循[ROS 2 开发人员指南]中有关 C/C++代码性能测试的建议(https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#performance)，并选择对每个版本而不是每个更改进行性能分析。

The performance tests of `rclcpp_components` are located in the [test/benchmark directory](https://github.com/ros2/rclcpp/tree/master/rclcpp_components/test/benchmark).

> `rclcppp_components`的性能测试位于[测试/基准目录](https://github.com/ros2/rclcpp/tree/master/rclcpp_components/test/benchmark).

Package and system level performance benchmarks that cover features of `rclcpp_components` can be found at:

> 包含`rclcpp_components`功能的软件包和系统级性能基准可以在以下位置找到：

- [Benchmarks](http://build.ros2.org/view/Rci/job/Rci__benchmark_ubuntu_focal_amd64/BenchmarkTable/)

> -[基准](http://build.ros2.org/view/Rci/job/Rci__benchmark_ubuntu_focal_amd64/BenchmarkTable/)

- [Performance](http://build.ros2.org/view/Rci/job/Rci__nightly-performance_ubuntu_focal_amd64/lastCompletedBuild/)

> -[性能](http://build.ros2.org/view/Rci/job/Rci__nightly-performance_ubuntu_focal_amd64/lastCompletedBuild/)

Changes that introduce regressions in performance must be adequately justified in order to be accepted and merged.

> 引入绩效倒退的变更必须充分合理，以便被接受和合并。

### Linters and Static Analysis [4.v]

`rclcpp_components` uses and passes all the ROS2 standard linters and static analysis tools for a C++ package as described in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#linters-and-static-analysis). Passing implies there are no linter/static errors when testing against CI of supported platforms.

> `rclcpp_components 使用并通过了[ROS 2 开发人员指南]中描述的 C++包的所有 ROS2 标准 linters 和静态分析工具(https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#linters-和静态分析）。通过意味着在针对支持平台的CI进行测试时，不存在linter/静态错误。

Currently nightly test results can be seen here:

> 当前夜间测试结果可在此处查看：

- [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rclcpp_components/)

> -[linux-arch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rclcpp_components/)

- [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rclcpp_components/)

> -[linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rclcpp_components/)

- [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rclcpp_components/)

> -[mac*osx*发布](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rclcpp_components/)

- [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rclcpp_components/)

> -[windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rclcpp_components/)

## Dependencies [5]

Below are evaluations of each of `rclcpp_components`'s run-time and build-time dependencies that have been determined to influence the quality.

> 下面是已确定影响质量的每个`rclcpp_components`的运行时和构建时相关性的评估。

It has several "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.

> 它有几个`构建工具`依赖项，这些依赖项不会影响包的最终质量，因为它们对公共库 API 没有贡献。

It also has several test dependencies, which do not affect the resulting quality of the package, because they are only used to build and run the test code.

> 它还有几个测试依赖项，这些依赖项不会影响包的最终质量，因为它们只用于构建和运行测试代码。

### Direct and Optional Runtime ROS Dependencies [5.i]/[5.ii]

`rclcpp_components` has the following runtime ROS dependencies:

> `rclcpp_components`具有以下运行时 ROS 依赖项：

#### `ament_index_cpp`

The `ament_index_cpp` package provides a C++ API to access the ament resource index.

> `ament_index_cpp`包提供了一个 C++API 来访问 ament 资源索引。

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ament/ament_index/blob/master/ament_index_cpp/QUALITY_DECLARATION.md).

> 它是**质量等级 1**，见其[质量声明文件](https://github.com/ament/ament_index/blob/master/ament_index_cpp/QUALITY_DECLARATION.md).

#### `class_loader`

The `class_loader` package provides a ROS-independent package for loading plugins during runtime

> `class_loader`包提供了一个独立于 ROS 的包，用于在运行时加载插件

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros/class_loader/blob/ros2/QUALITY_DECLARATION.md).

> 它是**质量等级 1**，见其[质量声明文件](https://github.com/ros/class_loader/blob/ros2/QUALITY_DECLARATION.md).

#### `composition_interfaces`

The `composition_interfaces` package contains message and service definitions for managing composable nodes in a container process.

> `composition_interfaces`包包含用于管理容器进程中可组合节点的消息和服务定义。

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/QUALITY_DECLARATION.md).

> 它是**质量等级 1**，见其[质量声明文件](https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/QUALITY_DECLARATION.md).

#### `rclcpp`

The `rclcpp` package provides the ROS client library in C++.

> `rclcpp`包提供 C++中的 ROS 客户端库。

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rclcpp/blob/master/rclcpp/QUALITY_DECLARATION.md).

> 它是**质量等级 1**，见其[质量声明文件](https://github.com/ros2/rclcpp/blob/master/rclcpp/QUALITY_DECLARATION.md).

#### `rcpputils`

The `rcpputils` package provides an API which contains common utilities and data structures useful when programming in C++.

> `rcpputils`包提供了一个 API，其中包含在 C++编程时有用的公共实用程序和数据结构。

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rcpputils/blob/master/QUALITY_DECLARATION.md).

> 它是**质量等级 1**，见其[质量声明文件](https://github.com/ros2/rcpputils/blob/master/QUALITY_DECLARATION.md).

### Direct Runtime non-ROS Dependency [5.iii]

`rclcpp_components` has no run-time or build-time dependencies that need to be considered for this declaration.

> `rclcpp_components`没有需要考虑的运行时或构建时依赖关系。

## Platform Support [6]

`rclcpp_components` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), and tests each change against all of them.

> `rclcppp_components 支持[REP-2000]中描述的所有第 1 层平台(https://www.ros.org/reps/rep-2000.html#support-层），并针对所有层测试每个更改。

Currently nightly build status can be seen here:

> 当前夜间构建状态可在此处查看：

- [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/rclcpp_components/)

> -[linux-arch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/rclcpp_components/)

- [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/rclcpp_components/)

> -[linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/rclcpp_components/)

- [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/rclcpp_components/)

> -[mac*osx*发布](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/rclcpp_components/)

- [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/rclcpp_components/)

> -[windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/rclcpp_components/)

## Security

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).

> 此软件包符合[REP-2006]中的漏洞披露政策(https://www.ros.org/reps/rep-2006.html).
