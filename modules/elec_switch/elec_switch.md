# 电气开关模块

## 使用范例

初始化时设置如下：

```c
ElecSwitch_Init_Config_s elecswitch_init_cofig = {
    .GPIOx = GPIO_Port,
    .GPIO_Pin = GPIO_Pin,
    .trigger_level = HIGH,
};
elecswitch_ins = ElecSwitchInit(&elecswitch_init_cofig);
```

需要添加新的开关时，请在elec_switch.h中添加开关所对应的GPIO口以便在app层调用初始化函数