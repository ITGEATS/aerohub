# PR 的 PX4 1.18 平台补丁

`as2-platform-px4-v1.18.patch` 是以下两个源码树之间的完整差异：

- 基线：官方 `as2_platform_pixhawk` 的 `fcba062219935acef69dfa009c8be376c707ee8a`，为生成该历史补丁时使用的基线。
- 目标：`https://github.com/Popc0rn7/as2_platform-aerohub.git` 的 `80f0f27a72efd053d029d3e6a76c3d5d473db6d2`，即原 PR 锁定的版本。

只在 AirSim / PX4 1.18 的构建副本上应用；主线 PX4 1.17 仍使用原来的补丁，不叠加两个版本的补丁。Docker 在 COPY 后应用；OHOS 在复制到隔离工作区后应用，不修改官方子模块。

此补丁涵盖参考时间戳、PVA 输出、命令回执和相关参数配置。补丁应用后的完整源码树已与目标提交逐文件核对相等；这不等于平台编译或飞行验证。

当前 AS2 与 Pixhawk 源码通过 Fork 子模块管理，仓库 URL 记录在根 `.gitmodules`，版本由父仓库 gitlink 锁定。本文件仅记录历史补丁的来源，应用前须核对构建入口与源码基线。
