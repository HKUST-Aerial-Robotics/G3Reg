# Registration Benchmarks
We evaluate the performance of our method on three datasets: KITTI, KITTI-360 and Apollo. If you want to know how to split the dataset, please see [LiDAR-Registration-Benchmark](https://github.com/HKUST-Aerial-Robotics/LiDAR-Registration-Benchmark).
## KITTI
[Download](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)

Folder structure
```angular2html
KITTI/odometry/data_odometry_velodyne/dataset
└── sequences
    ├── 00
    ├── 02
    ├── 05
    ├── 06
    ├── 08
    ├── 09
    └── 10
```
+ Please update the YAML file by replacing the dataset_root variable with YOUR_PATH.
```shell
# KITTI-10m
./bin/reg_bm configs/kitti_lc_bm/fpfh_3dmac.yaml configs/datasets/kitti_10m/test_10.txt
./bin/reg_bm configs/kitti_lc_bm/fpfh_pagor.yaml configs/datasets/kitti_10m/test_10.txt
./bin/reg_bm configs/kitti_lc_bm/fpfh_ransac.yaml configs/datasets/kitti_10m/test_10.txt
./bin/reg_bm configs/kitti_lc_bm/fpfh_teaser.yaml configs/datasets/kitti_10m/test_10.txt
./bin/reg_bm configs/kitti_lc_bm/fpfh_quatro.yaml configs/datasets/kitti_10m/test_10.txt

./bin/reg_bm configs/kitti_lc_bm/gem_3dmac.yaml configs/datasets/kitti_10m/test_10.txt
./bin/reg_bm configs/kitti_lc_bm/gem_pagor.yaml configs/datasets/kitti_10m/test_10.txt
./bin/reg_bm configs/kitti_lc_bm/gem_ransac.yaml configs/datasets/kitti_10m/test_10.txt
./bin/reg_bm configs/kitti_lc_bm/gem_quatro.yaml configs/datasets/kitti_10m/test_10.txt
./bin/reg_bm configs/kitti_lc_bm/gem_teaser.yaml configs/datasets/kitti_10m/test_10.txt
```
```shell
# KITTI-loop
./bin/reg_bm configs/kitti_lc_bm/fpfh_3dmac.yaml configs/datasets/kitti_lc/test_0_10.txt
./bin/reg_bm configs/kitti_lc_bm/fpfh_pagor.yaml configs/datasets/kitti_lc/test_0_10.txt
./bin/reg_bm configs/kitti_lc_bm/fpfh_ransac.yaml configs/datasets/kitti_lc/test_0_10.txt
./bin/reg_bm configs/kitti_lc_bm/fpfh_teaser.yaml configs/datasets/kitti_lc/test_0_10.txt
./bin/reg_bm configs/kitti_lc_bm/fpfh_quatro.yaml configs/datasets/kitti_lc/test_0_10.txt

./bin/reg_bm configs/kitti_lc_bm/gem_3dmac.yaml configs/datasets/kitti_lc/test_0_10.txt
./bin/reg_bm configs/kitti_lc_bm/gem_pagor.yaml configs/datasets/kitti_lc/test_0_10.txt
./bin/reg_bm configs/kitti_lc_bm/gem_ransac.yaml configs/datasets/kitti_lc/test_0_10.txt
./bin/reg_bm configs/kitti_lc_bm/gem_teaser.yaml configs/datasets/kitti_lc/test_0_10.txt
./bin/reg_bm configs/kitti_lc_bm/gem_quatro.yaml configs/datasets/kitti_lc/test_0_10.txt
```

## KITTI-360
[Download](https://www.cvlibs.net/datasets/kitti-360/)

Folder structure
```angular2html
── KITTI-360 (YOUR KITTI-360 PATH)
│   ├── calibration
│   ├── data_3d_raw
│   └── data_poses
```
+ Please update the YAML file by replacing the dataset_root variable with YOUR_PATH. 
+ You can replace "test_0_10.txt" as "test_10_20.txt" or "test_20_30.txt".
```angular2html
./bin/reg_bm configs/kitti360_lc_bm/fpfh_3dmac.yaml configs/datasets/kitti360_lc/test_0_10.txt
./bin/reg_bm configs/kitti360_lc_bm/fpfh_pagor.yaml configs/datasets/kitti360_lc/test_0_10.txt
./bin/reg_bm configs/kitti360_lc_bm/fpfh_quatro.yaml configs/datasets/kitti360_lc/test_0_10.txt
./bin/reg_bm configs/kitti360_lc_bm/fpfh_ransac.yaml configs/datasets/kitti360_lc/test_0_10.txt
./bin/reg_bm configs/kitti360_lc_bm/fpfh_teaser.yaml configs/datasets/kitti360_lc/test_0_10.txt

./bin/reg_bm configs/kitti360_lc_bm/gem_3dmac.yaml configs/datasets/kitti360_lc/test_0_10.txt
./bin/reg_bm configs/kitti360_lc_bm/gem_pagor.yaml configs/datasets/kitti360_lc/test_0_10.txt
./bin/reg_bm configs/kitti360_lc_bm/gem_quatro.yaml configs/datasets/kitti360_lc/test_0_10.txt
./bin/reg_bm configs/kitti360_lc_bm/gem_ransac.yaml configs/datasets/kitti360_lc/test_0_10.txt
./bin/reg_bm configs/kitti360_lc_bm/gem_teaser.yaml configs/datasets/kitti360_lc/test_0_10.txt
```

## Apollo
[Download](https://developer.apollo.auto/southbay.html)

Folder structure
```angular2html
Apollo
└── TestData
    ├── BaylandsToSeafood
    ├── ColumbiaPark
    ├── HighWay237
    ├── MathildaAVE
    ├── SanJoseDowntown
    └── SunnyvaleBigloop
```
+ Please update the YAML file by replacing the dataset_root variable with YOUR_PATH.
+ You can replace "test_0_10.txt" as "test_10_20.txt", "test_20_30.txt" or "test_30_40.txt".
```angular2html
./bin/reg_bm configs/apollo_lc_bm/fpfh_3dmac.yaml configs/datasets/apollo_lc/test_0_10.txt
./bin/reg_bm configs/apollo_lc_bm/fpfh_pagor.yaml configs/datasets/apollo_lc/test_0_10.txt
./bin/reg_bm configs/apollo_lc_bm/fpfh_quatro.yaml configs/datasets/apollo_lc/test_0_10.txt
./bin/reg_bm configs/apollo_lc_bm/fpfh_ransac.yaml configs/datasets/apollo_lc/test_0_10.txt
./bin/reg_bm configs/apollo_lc_bm/fpfh_teaser.yaml configs/datasets/apollo_lc/test_0_10.txt

./bin/reg_bm configs/apollo_lc_bm/gem_3dmac.yaml configs/datasets/apollo_lc/test_0_10.txt
./bin/reg_bm configs/apollo_lc_bm/gem_pagor.yaml configs/datasets/apollo_lc/test_0_10.txt
./bin/reg_bm configs/apollo_lc_bm/gem_quatro.yaml configs/datasets/apollo_lc/test_0_10.txt
./bin/reg_bm configs/apollo_lc_bm/gem_ransac.yaml configs/datasets/apollo_lc/test_0_10.txt
./bin/reg_bm configs/apollo_lc_bm/gem_teaser.yaml configs/datasets/apollo_lc/test_0_10.txt
```
# Front-end Benchmarks
```shell
# KITTI-loop
./bin/matching_bm configs/kitti_lc_bm/fpfh_pagor.yaml configs/datasets/kitti_lc/test_0_10.txt
./bin/matching_bm configs/kitti_lc_bm/gem_pagor.yaml configs/datasets/kitti_lc/test_0_10.txt
```