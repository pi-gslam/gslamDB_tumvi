# gslamDB_tumvi


This project implements the GSLAM dataset plugin for dataset TUMVI: https://vision.in.tum.de/data/datasets/visual-inertial-dataset .


## Compile and Install

1. Please compile and install GSLAM: https://github.com/zdzhaoyong/GSLAM
2. Please compile and install PICMake: https://github.com/zdzhaoyong/PICMake
3. Compile this project with cmake

```
mkdir build;
cd build;
cmake ..;
make;
sudo make install
```

## Play the dataset with GSLAM

First download a sample dataset:

https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-calib-cam1_512_16.tar

Decompress and play with GSLAM:

```
gslam qviz play -dataset dataset-calib-cam1_512_16/mav0/.tumvi
```

