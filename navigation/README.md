# 安装编译依赖

### 1、安装nlopt

用于路径优化的优化库

```
sudo apt install libnlopt-dev
```

### 2、安装osqp

用于mpc（模型预测控制）的依赖

```
git clone --recursive https://github.com/oxfordcontrol/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/usr ..
cmake --build .
sudo cmake --build . --target install
```

### 3、安装ompl

一些弧线依赖库

```
sudo apt install libompl-dev
```