## Params测试用例
> 开发人员：@wuhuabo 测试人员：@wuhuabo @chenmingjian

### 1.参数设置和获取测试用例
> 功能描述：测试参数设置和获取是否正常-getParam setParam

| 编号 | 输入/动作                                               | 期望的输出/响应                  | 实际情况 |
| ---- | ------------------------------------------------------- | -------------------------------- | -------- |
| 1    |获取未设置的int参数| 调用getParam返回false| 符合     | 获取失败返回false
| 2    |先设置int参数10，再获取对应参数| 先调用setParam，调用getParam返回true,并且参数不错 | 符合     |
| 3    |获取未设置的bool参数| 调用getParam返回false| 符合     | 获取失败返回false
| 4    |先设置bool参数true，再获取对应参数| 先调用setParam，调用getParam返回true,并且参数不错 | 符合     |
| 5    |获取未设置的double参数| 调用getParam返回false| 符合     | 获取失败返回false
| 6    |先设置double参数54.568，再获取对应参数| 先调用setParam，调用getParam返回true,并且参数不错 | 符合     |
| 7    |获取未设置的string参数| 调用getParam返回false| 符合     | 获取失败返回false
| 8    |先设置string参数abc，再获取对应参数| 先调用setParam，调用getParam返回true,并且参数不错 | 符合     |