## WorldMapData测试用例
> 开发人员：@wuhuabo 测试人员：@wuhuabo @chenmingjian

### 1.通过地图文件获取地图测试用例
> 功能描述：测试通过地图文件获取地图是否正常-readMapFromYaml

| 编号 | 输入/动作                                               | 期望的输出/响应                  | 实际情况 |
| ---- | ------------------------------------------------------- | -------------------------------- | -------- |
| 1    |给定已知地图文件路径| 调用readMapFromYaml返回true| 符合     | 获取失败返回false