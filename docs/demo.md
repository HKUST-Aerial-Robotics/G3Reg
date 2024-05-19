## Demo
```angular2html
sudo apt install pcl-tools
```
### Debug Segmentation
```angular2html
./bin/demo_seg configs/apollo_lc_bm/gem_pagor.yaml examples/data/velodyne/source.pcd
```
### Try Livox Point Cloud
```angular2html
# Try Livox point cloud
./bin/demo_reg configs/hit_ms/gem_pagor.yaml examples/data/livox/source.pcd examples/data/livox/target.pcd
pcl_viewer -fc 255,180,0 Log/demo/source_transformed.pcd -fc 0,166,237 Log/demo/target.pcd -bc 255,255,255
```
### Try Velodyne Point Cloud
```angular2html
# Try Velodyne point cloud
./bin/demo_reg configs/apollo_lc_bm/gem_pagor.yaml examples/data/velodyne/source.pcd examples/data/velodyne/target.pcd
pcl_viewer -fc 255,180,0 Log/demo/source_transformed.pcd -fc 0,166,237 Log/demo/target.pcd -bc 255,255,255
```