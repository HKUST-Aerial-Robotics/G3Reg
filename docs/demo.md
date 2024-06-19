## Demo
```angular2html
sudo apt install pcl-tools
```
### Debug Segmentation
```angular2html
./bin/demo_seg configs/apollo_lc_bm/gem_pagor.yaml examples/data/velodyne/source.pcd
# If visualization does not work, comment "visualizeCloud" and try to open the pcd file with pcl_viewer
pcl_viewer Log/demo/source_colored.pcd -ps 2 Log/demo/quadric_colored.pcd -ps 3.5 -bc 255,255,255
```
![2024-05-19_17-32](https://github.com/HKUST-Aerial-Robotics/G3Reg/assets/21232185/aad71d54-ceba-4742-b165-8502b2ed57c2)

### Try Livox Point Cloud
```angular2html
# Try Livox point cloud
./bin/demo_reg configs/hit_ms/gem_pagor.yaml examples/data/livox/source.pcd examples/data/livox/target.pcd
pcl_viewer -fc 255,180,0 Log/demo/source_transformed.pcd -fc 0,166,237 Log/demo/target.pcd -bc 255,255,255
```
![2024-05-19_17-33](https://github.com/HKUST-Aerial-Robotics/G3Reg/assets/21232185/1e96e5dc-b592-4bf6-80a3-1a93da2adacf)

### Try Velodyne Point Cloud
```angular2html
# Try Velodyne point cloud
./bin/demo_reg configs/apollo_lc_bm/gem_pagor.yaml examples/data/velodyne/source.pcd examples/data/velodyne/target.pcd
pcl_viewer -fc 255,180,0 Log/demo/source_transformed.pcd -fc 0,166,237 Log/demo/target.pcd -bc 255,255,255
```
![2024-05-19_17-34](https://github.com/HKUST-Aerial-Robotics/G3Reg/assets/21232185/5b574ec1-401f-486a-b7db-d3876ecb80c7)
