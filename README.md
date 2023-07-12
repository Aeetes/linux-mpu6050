# linux-driver-mpu6050

linux driver for mpu6050

## dts

- 在i2c控制器下追加节点

```
mpu6050:mpu6050@68 {
    compatible = "aeetes,mpu6050";
    reg = <0x68>;
    status = "okay";
};
```

## make

- 修改KERNELDIR为内核目录

```
KERNELDIR := /home/aeetes/workspace/mys-6ulx-iot/myir-imx-linux
```



## demo

- mpu6050Demo为测试程序，读原始ADC和实际值