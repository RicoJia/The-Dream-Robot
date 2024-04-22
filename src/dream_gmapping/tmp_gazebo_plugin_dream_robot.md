# Gmapping

========================================================================
## Process 
========================================================================
### Process Start Week of Feb 17
1. get_screw_displacement(prev, left_odom, right_odom) -> v, theta. odom_transform(inverse of icc) (D)
    - icp align T_init_guess to screw displacement; Test: a global point cloud, check if there's memory leaks
    - Test for a plane: 360 at distance 1m away. (D)
2. If odom, angular distance is not enough, skip. (D)
- Is there a chance that we can get rid of the vector of past poses?? Because we just care about the obstacle it generates. Just don't save, but let's keep the vector

### Scan Match Week of Feb 24
1. scan_match_success, icp_corrected = icp(prev_odom, last_scan, current_scan, T_init)
    - how does PCL do it, expensive? (D)
    - get_point_cloud_in_world_frame(robot_pose, scan_msg);, test (D)
2. If scan_match_fail:
    for each particle:
        1. draw from motion (D)
        2. update score and motion pose ()
3. if scan_match successful
    for each particle
        1. new_pose *= icp
        2. update_particle_pose


### motion model drawing (prev_odom, v, theta), (D), Week of feb 24
for each particle:
2. generate_noise
3. icc(v, theta) -> Tinit, score
    1. get x, y, theta
    2. apply icc

#### update_particle_pose
1. sample K. for each K particle_score = score(k_pose, laserscan)
    1. for each pose in kernel K: (pixelize point cloud?)
        - Do I need the naive world frame one? We want to store pc in pixels.  - transform -> world pixel - world -> transform (center)
    2. `score = motion model score * likelihood comparison (laser point needs to be transformed to pixels already.).`. Likelihood comparison: 2000 points `(2000*k look ups + if);` See scanmatcher.h, score. "Fullness". For each beam: 
        1. compare ip_free and if_hit at the same time. 
        2. move around the kernel, find the closest occupied cell, and its distance, mu. The score of that beam is exp(-1 * mu * mu/sigma).         3. Then total score of k_pose is prod(exp(-1 * mu * mu/sigma)) * p(motion_model)
    3. Corner cases: 
        1. Beam too short: 
        2. Beam too long:

2. Find the best K.

### Resample
1. calculate neff: normalize weights: w/total_w
2. if neff is too small:
    1. Generate cdf across all particles, from 0-1.
    2. Generate N uniformly distributed numbers. where does that land? You reproduce 
    3. Store the most voted particle
3. If neff is not small: go through the normalized weights and find the heaviest one
    - Neff is a measure of how scatterred our guesses are. They could be all bad

### update map
1. create_map(): go through a particle's dict, if count is higher than threshold, put on the map
2. publish map

### Dockerization
- pcl

### Real Commision
- orientation of the laser scanner: [-pi, pi]? Where is the beginning?
========================================================================
## Reasoning 
========================================================================
### Weight Updates 
1. Each particle has its own map. 

2. Where does particles' variablity come from? 
    1. Random sampling from motion model
    2. Then, you update the estimate with the ICP-corrected pose and select the best one in K samples 
    3. Resampling doesn't add randomness here

#### Observation Model Evaluation
1. How to evaluate sensor model, like the likelihood field model?
    - In gmapping, you find a laser reading's endpoints on the map frame. Then, find each endpoint's distance to obstacles, then plug it into Gaussian Likelihood model. 
        - This is done in `resample() -> Particle (gridslamprocessor.h) -> ScanMatcherMap map -> Map<PointAccumulator,HierarchicalArray2D<PointAccumulator> >`
    - **Bmapping is only sticking to one map. Which is not sufficient for multiple particles with different trajectories**.        

2. Then, what's the problem with using a single map's likelihood field? 
    - If our particles are always close, then this might be "sharable" in some way. However, particles could be way different. See Particle Creation for where particles' variability comes from

3. How does ROS gmapping `likelihoodAndScore()` work (scanmatcher.h):
    - It calculates where `phit` is, then it checks a kernel around it. If there's occupancy in that grid,  it p will be determined by the distance to that. Otherwise, a default no hit is added.

4. How to efficiently store a likelihood field for each particle, then to calculate p(z|m,x)?
    - KD tree or a quad tree. KD tree (PCL) vs quad tree? Not selected because they cannot be used to construct a map directly; also, searching for min dist is o(nlog(n)) for n points
    - **what about for each particle: {<x,y>: count}? Looks good**
        - can create a map according to that
        - given laser end point, we can search for neighbors in vincinity in this dictionary. 

### Resample
    
- How do we store trajectory points in each particle?
    - its lifetime should be managed by shared pointer.
    - Copy unique_ptr, and shared_ptr?

### Update Mapping
- How to update mapping?
    1. find the best particle
    2. track down its nodes, project scans on to it.
    3. Count how many times each ray has been casted

- does this new sequence correspond with the paper? bmapping? gmapping? 
    - in paper: updating a particle's pose involves sampling K poses, generate a new distribution from it, then draw from the distribution again. This is equivalent to "updating with the scan match"
    - From Bmapping, 
        - We do a global scan match. That's good enough as an estimate.
        - Since it's only using a single map, adding scan to each particle will be irrelevant.
        - However, bmapping really is calculating k and each mu
    - From ROS Gmapping:
        - My understanding of why the paper takes mu and sigma then resample, really is to get an estimate given the k poses, and their probablity of the observation model and odometry. - process_scan(): 
        - In ROS gmapping, it's done by: moving left, right, ...  with delta. Delta becomes smaller and smaller. So, it's NOT considering the score from motion model.
        is this the real time correlation paper? No, the real time paper specified a multi-layer look up, with a low res & high res map.
========================================================================
## Engineering: Test as you go
========================================================================

### Pre-Trip Processing
1. Record bag from sim: scan, wheel odom (D)
2. test showing memory management drops to 0, with copy (D)
3. Base structure slam Node: listening on scan, joint_states
    - Create Particle with weights (D)
    - Test message filter (D)
    - Import lib.  map: occupancy grid, odom is published on odom.
    - odom path, slam path
    - TF:
        - map -> odom
        - odom -> base_link
4. script that publishes a laser scan; encoder reading; ()
    - See if you can get some more readings
        - Like, you can have a simple shape, like a circle. zero encoder reading
    - find a map

### Week of March 5: Testing And Bug Fixes

#### Odom Reorg
odometry workflow: ticks -> tick wrap -> wheel angle |-> unwrapped angle -> tf (wheel dist and icc required)

1. Odom class, DreamOdometer (D)
    - listen to wheel_pos, outputs transform to /odom->/base_link.
    - get_2D_screw_displacement()
2. In DreamGmapper, listen to tf.
    - angle wrap the delta, update the last odom with the current odom
    - if delta_odom has passed threshold, we need continue with laser scan
3. In test, spawn an instance of the odom class
4. In publish map, output tf /map -> /odom
5. screw_displacement_2d_to_body_frame_transform()

### Bug List
- Done
    - There was a silent segfault. Shouldn't cause problems? particle size in test wasn't right; segfault from vector; assignment wasn't right (D)
        cloud_in_world_frame = DreamGMapping::get_point_cloud_in_world_frame(new_pose_estimate,
                                                    cloud_in_body_frame);?
    - why 358? (D)
    - point cloud size isn't right. Fix? size = 0 (D)
    - C++ exception with description "vector::_M_range_check: __n (which is 18446744073709551602) >= this->size() (which is 441)" thrown in the test body: this is because the wall is infinitely long. One way is to make a wall that's 1m long. (D)
    - Why odom is at the bottom of the map, instead of at the center of the circle? Because you selected odom, but there's no tf for map-> odom. If selecting map, there will be.
    - Why there's points beyond the line? (not sure, changing the range to smaller made this issue go away)
    - publish map to odom tf (see the code)
    - map to base not right.
        - see particles across all runs, make it 1
        - expect map to baselink to be identity
            - it's not identity? (because we need to explicitly cast product inve * to eigen matrix)
            - See the odom itself: the wall itself is great
            - ICP didn't do a good job? (D)
                ```
                icp output: 0.984414 -0.17591        0 0.207735
                 0.17591 0.984414        0 0.120956
                       0        0        1        0
                       0        0        0        1
                ```
            - what if we do icp on world frame points? No need, because two point clouds are already transformed into the same body frame
            - observation model scoring is based on angle of beam, because you want to evaluate along each line: the phit, and the pfree
                1. observation model: change to p_hit only (D)
                2. lint (D)

                0. Create a toggle. (skip_invalid_scans), signature (D)
                1. fill_point_cloud (D)
                2. When transforming to world frame, just do the (D)
                3. adding the lines to the range_max ones?
                    - observation model: just compare the valid ones. FUTURE IMPROVE: compare observation model with free points
                    - reorg:
                        1. add a vec cloud_occupied_vec, (then you have 2 vecs)
                        2. scan message: (advantage: clean interface.)
            - get rid of cloud_in_world_frame_vec
    - map to base not right. (D)
    - Make sure the max ranges are not registered, but the line all the way up to them are (D)
- Reorg wheel_pos_topic?
    - catkin make 
    - catkin make test
    - why constexpr const char* WHEEL_POS_TOPIC = "wheel_positions";?
    - relaunch and check if subscription is done

- Warning: TF_OLD_DATA ignoring data from the past (Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained) for frame base_link at time 522.938000 according to authority unknown_publisher (use_sim_time, --clock, -r 1)

- Exception thrown:Could not find a connection between 'odom' and 'base_link' because they are not part of the same tree.Tf has two or more unconnected trees.
    The current list of frames is:
    Frame left_wheel exists with parent chassis.
    Frame right_wheel exists with parent chassis.
    Frame base_link exists with parent map_ground_truth.
        - we need map_ground_truth as parent frame

- odom -> based_link: 
    - remove map and base link name
    - Launch sim? add in readme
    ```
    source src/dream_mobile_platform/dream_feature_flags
    SIM=true roslaunch dream_mobile_platform low_level_drivers.launch
    ```

    - record
    rosbag record --bz2 /dream/scan /dream/wheel_pos /dream/map_groud_truth_pose

- Try bringing up the bag again
    - rosrun tf tf_echo odom base_link
    - add rviz

- configure gmapping
    - verify on rviz, there's tf (D)
    - how to run this: roslaunch dream_gmapping dream_gmapping.launch sim:=true (D)
    - why doesn't echo laser scan: abs

- neff?: score is too low. if it's empty, then what? should give it a non-zero low score. Bmapping?
    - contains (D)
    - line end point was not gut. end point in world is too large: because there were an outlier in scan.
    - replay the bag
        - launch file: odom, dream_gmapper; 
        - takes in /dream/scan /dream/wheel_pos

- bag does not provide wheel_pos? - resetting the gazebo world just eliminates the publishers
- scan to base_link tf?
    - crashed? gdb; then run it and see (D)
        - set_target_properties(dream_gmapper PROPERTIES COMPILE_FLAGS "-g")
        - launch-prefix="gdb -ex run --args"
    - Why so many empty lines? (odom not far enough)
    - add scan tf lookup: 
    - apply scan_to_body_frame

- map not published correctly?
- Point not found? Give it a score? Check probablistic robotcs


### Improvements
- Neff is a measure of how scatterred our guesses are. They could be all bad
- Why does particles not picking up the right pose?
    - Resampling is not eliminating bad particles? Test: rotation, with lots of particles.
        - penalty for not picking up the ray seems  not low enough;
        - overall score is still low. A better metric?
            a not found score : 1.02^-15.12 = 0.74 
    - Motion model is kind of bad. If it's bad,  Map that has been built is also bad (TODO)
        - using correction_kernel = 3 does seem to bring up an improvement. Going up may be saturated
launch-prefix="valgrind --tool=callgrind --callgrind-out-file='callgrind.dream_gmapping.%p'"

1. ROS Gmapping Live Investigation:
    - time for the whole trip
        - motion validation
            - 1 particle, zero motion drawing is still ok. So with motion drawing, it's an add on. Its correction is really the key here.
                ```
                correction: (-3.09696,0.773438,1.34169) -> (-3.16102,0.803125,1.19169)
                ```
            - is our 0.05 too coarse?? Gmapping's scan matching is the biggest factor that made it successful. It performs **(gradient descent)**
                1. have 0.05 as gain. 
                2. move in 6 directions from the current pose, find the best scoring onem then set it to current pose
                3. from the new current pose, if currentScore >= best score, keep going. Otherwise, a_local, l_local is divided in halves 
                4. Repeat 2

            - Scoring
                - what's the laser beam? Could it be "if 1 miss is costing too much? So kernel misses do not triamph?"

    - Secondary factors
        - Why particles don't change much
            - I barely see any resampling  
            - Are we using motion model at all?
            - How do we correct? score is over 300
            - scan is [max, ... ]
        - **compare occupied cells, where what's each cell's hit, etc. if they are obstacle between each run** 
            - map cells from the current laser scan corresponds well with the laser points
            - map cells from the first run could be used for good localization
        - How are obstacles added? Why do they not vanish
                - register scan
                ```
                // scanmatcher.cpp, this could be interesting
                PointAccumulator &cell = map.cell(line.points[i]);
                ```
                - void SlamGMapping::updateMap(const sensor_msgs::LaserScan &scan) TODO

        - How noise is added? (not major)
            - motion model (motion model is effective. and the motion noises are small. But this is not the foundamental reason for stable mapping)
            - How is motion model useful

    - TODO List
        - particle update 
            - do it every map update interval (5s) as well. In other times, we just display odom (no /map -> /odom update). So for search kernel = 1, particle num = 30, a performance of 0.5s per update is good. 
            - particle poses and weight:
                1. scan match
                     - step 1
                     (-0.000910683,0.00305944,-1.57053), 331.994
                     (0.000321912,0.00464879,-1.56982), 332.019
                     - step 2
                         (-0.132281,0.0134467,-1.57307)
                         (-0.131683,0.0137762,-1.57291)

        - observation scoring:
            - add exp(-1./m_gaussianSigma*bestMu*bestMu), where m_gaussianSigma is 0.05, and bestMu is <0.01
            - skip rays beyond usable range (max_range, or min_range) kernel size is 1
            - TODO: find the best mu, compare that with your current mu!!!

### Optional Improvement 1: Motion Model Reorg

1. Add noise to T directly, using something similar to `drawFromMotion` That'd be a motion model; Apply that to each particle
2. Do ICP, get T_icp, appluy that to each particle 
3. Optimize: larger pose_estimates kernel. and maybe a larger beam scan kernel?
    - motion model is needed, especially with random noise. That gives more randomness to help counter balance odometer drift, and icp inaccuracies

- Optional: P(x|z) This will really come from site measurement: 1%(3m), 2%(3-5m), 2.5% (5-12m)

7. Gitlab CI
8. docker container update Debian package? github?

========================================================================
##  Gmapping Code reading
========================================================================
1. `SlamGMapping gn;`
    ```cpp
    map_to_odom_, laser_count_, scan_filter_sub_, scan_filter_, transform_thread_gn.startLiveSlam(); 
    sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
    transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));
    ```

2. `laserCallback`: orientation of the laserscan is crucial.
    1. `initMapper()`
        ```cpp
        // Get the laser's pose, relative to base.
        // check if laser scan is mounted planner, and fail
        // check laser is mounted up/down? 

        // TODO: what are these?
        gsp_ = new GMapping::GridSlamProcessor();
        GMapping::SensorMap smap;
        gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
        GMapping::sampleGaussian(1,seed_);
        ```

    2. `addScan(*scan, odom_pose)`
        ```cpp
        getOdomPose(gmap_pose, scan.header.stamp)
        // filter out the short ones.

        // this sets the scan with range_max, and odom pose
        reading.setPose(gmap_pose);
        // real particle update 
        return gsp_->processScan(reading); // (gridslamprocessor.cpp)
            if (travelled enough distance / angular || time || !m_count)
            //write the state of the reading and update all the particles using the motion model
            for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
                OrientedPoint& pose(it->pose);
                pose=m_motionModel.drawFromMotion(it->pose, relPose, m_odoPose);
            }

            // check if new odom_pose and the old pose have a huge jump
            scanMatch(plainReading); //gridslamprocessor.hxx 
                // for each particle 
                score=m_matcher.optimize(corrected, it->map, it->pose, plainReading); // scanmatcher.cpp
                    - go over left, right, ... motions, return the best score that match AGAINST THE ENTIRE MAP. Then, "do the same motion"with smaller alpha and ldeta until your value gets smaller. the corrected pose is the best pose.
                m_matcher.likelihoodAndScore(s, l, it->map, it->pose, plainReading); //scanmatcher.h
                it->weight+=l;
                // we are adopting the scan matched result as the mean. TODO: tnode is available at this point? Yes, see below
                it->pose=corrected;
                // if no match is found,  use odom

            updateTreeWeights(false); // gridslamprocessor_tree.cpp
                normalize();
                // add up child Tnodes weights
                propagateWeights(); 

            resample(plainReading, adaptParticles, reading_copy); //gridslamprocessor.hxx??
                - figure out which particles to keep, then make copies of them
                - register scans to each particle's maps
                - create new Tnode, with the current pose for next iteration's update. the new tnode will be the tail
                - delete old particles with their tnode, free up their spaces.
            updateTreeWeights(false);
        ```

3. `GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;`
    ```cpp
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));
    // Update map_to_odom
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    // Update map, if it's time, or haven't got map yet
    ```

4. `updateMap(*scan); //slam_gmapping.cpp`
    ```cpp
    TODO: how are the tree built for a particle?
      GMapping::ScanMatcher matcher;
      GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
      GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, delta_);
      // add readings from the beginning
      for(GMapping::GridSlamProcessor::TNode* n = best.node; n; n = n->parent){
        // see which regions need to be updated
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
            // 1. find the current pose on the map
            // 2. calculate bounding box resize map
            // 3. hit points of the rays
        matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
            // 1. from each hit point back to bresenham ray tracing. Update smap ???
            cell.update(false, Point(0,0));
            // available, what if it was previously  an obstacle cell?
            map.cell(p1).update(true, phit);
    }
    // Resize the window if necessary
    publish map 
    ```

### Lower level datastructure
```cpp
grid/map.h
namespace{
template <class Cell, class Storage, const bool isClass=true>class Map{
      const Cell& Map<Cell,Storage,isClass>::cell(const IntPoint& p) const {
      // m_storage = Storage, which is HierarchicalArray2D
      return m_storage.cell(p);
}
    class Cell{

    }
}
// smmap.h
typedef Map<PointAccumulator,HierarchicalArray2D<PointAccumulator> > ScanMatcherMap;
struct PointAccumulator{
    void PointAccumulator::update(bool value, const Point& p){
    if (value) {
        acc.x+= static_cast<float>(p.x);
        acc.y+= static_cast<float>(p.y);
        n++;
        visits+=SIGHT_INC;
    } else
        visits++;
}
```
- When you query a map's occupancy:    // smmap.h. 
```cpp
// RJ: effectively, if n/total_visits < thre, it's not an obstacle
    inline operator double() const { return visits?(double)n*SIGHT_INC/(double)visits:-1; }
}
```

- `drawFromMotion()` motionmodel.cpp
    ```cpp
	double lm=linearMove  + fabs( linearMove ) * sampleGaussian( srr ) + fabs( angularMove ) * sampleGaussian( str );
    ```

- `likelihoodAndScore(s, p)` (scanmatcher.h), s is the total weight
    ```cpp
    // get the laser pose  from p
    for each laser beam:
        - get its end point phit, and get xx*yy neighbors, pr. 
        - each pr has a slight offseted point, pf. 
        - if pr is occupied, but pf is not, then we think we have a potential candidate for mu, where mu is phit - pr.
        - find the lowest mu among all phit.
        // so this is a fixed sigma.
        double f=(-1./m_likelihoodSigma)*(bestMu*bestMu);
        l+=(found)?f:noHit;
    ```
    - so l is the measured by how close the closest map points to each hit point, in total.
    - `normalize()` in `gridslamprocessor.hxx`, using log trick to reduce numerical instabilities
        ```
        exp(gain*(it->weight-lmax))
        ```

- `updateTreeWeights` (gridslamprocessor_tree.cpp)
    ```
    normalize(); // gridslamprocessor.hxx
       //normalize the log m_weights ??? TODO
       get neff from m_weights, eventually from m_particles->weight_
    resetTree();
    propagateWeights(); 
    ```

## Bmapping
scan match using the odom for each particle.if fails:
    1. sampleMotionModel 
    2. Update the likelihood p(z|m, x)
        - for each beam endpoint
If the matcher succeeds, update its pose with ICP result:
    1. sample a bunch of poses around mode (sampleMode)
    2. score the observation and likelihood field. gaussianProposal
    3. Then, really calculate mu and sigma from the k samples. Then, draw from it. TODO how to calculate the mean again???

- ICP
    ```cpp
    pclICPWrapper (cloud_alignment.cpp) ("find scan match from the old reading and the new reading")??
    ```

========================================================================
## Reading List
========================================================================

1. https://www.guyuehome.com/35772
2. Paper: http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti07tro.pdf
    - What is Rao-Blackwellized Particle Filter - the separation
    - what is scan matching: find relative pose between two poses (map and current scan)
3. UBC wiki: https://wiki.ubc.ca/Course:CPSC522/Rao_Blackwellized_Particle_Filtering
    - RL, inference, Deep neuralnets
4. Boston's Bmapping: https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/bmapping
5. occupancy grid mapping: https://www.guyuehome.com/35771

6. Kamath Project, similar to yours(2011 - Present) but no gmapping
    https://adityakamath.github.io/projects/

========================================================================
## Learnings 
========================================================================
 
### C++
- CMake
    - If I have a custom library that uses eigen3, do I need to include eigen3 in my other projects?
    - 
- Use of a void pointer? "Universal pointer"
- We want to delete and copy particles. How to design our data structure to efficiently and rapidly process map data?
    - We need kd tree vs map to store obstacles seen on a map so far. Our map is 100m^2, accuracy 5cm x5cm
        - Memory usage:
            - KD tree: 8 bytes * (40m/5cm) * 2(times more obstacles) * 40 (# particles) = 2 MB
            - Map: 8 bytes * (100 m^2/25cm^2) * 40 = 1.2 * 10 ^ 7 = 12 MB, that's still not likelihood tho.
        - Performance:
            - Map: -> scan matching score: need likelihood score at each pixel: when registering, you can either only update neighbor cells, or you have to ray trace all cells and update them.
                1. Need to transform laser endpoints to the corresponding frame
                2. update: o(n)
                3. Query: o(1)
            - KD tree: when registering laser end points, you have to 
                1. Need to transform laser endpoints to the corresponding frame
                2. Inserting into tree, o(n*log(n)) for inserting n points. However, for our target space, n (40m/5cm) * 2(times more obstacles) = 1.6*10^4. 
                3. When querying, it's o(nlog(n)). 
            - openvdb
        - Updatemap performance 
            - KD Tree: cannot be used for the counting method. We still need to go over all laser scans and traj poses to count how many times
            a point is hit.
            - Map: No direct help either. 
        - So, KD tree could be a good choice, if: **sqrt is fast enough; KD tree for inserting 360 into the tree, and searching is fast enough**
- What's row major: when you flatten a 2D matrix -> vector, you do it by rows
- what is ESDF: euclidean signed distance field
- Does c++ struct have default ctro?
    - will provide one `Pose()` if you don't define one. 
    - However, built-in types, like int, float etc. Will have garbage values
    - `Pose p = {1,2,3}` is an **aggregate type**, which uses aggregate initialization, C++11 +
    - for aggregate type, ```std::make_shared<T>(args)``` doesn't work. Also, ```std::make_shared<T>({args})``` is an initializer list, which requires an explicit ctor.
    - Once you have a customly defined ctor, the class will not be an aggregate type. However, ```std::make_shared<T>(args)``` should work with all args perfectly forwarded.
- How's copy ctor used?
    ```
    Particle get_copy(){
        return Particle
    }
    ```
- How to create a new unique ptr with another ptr's object?
    - make sure copy ctor exists
    - `auto ptr = std::make_unique<T>(*ptr)`

- If all your ctors and copy ctor, dtors are default, your don't need them. One "trick case" is this:
    ```
    struct Particle{
        double _weight;
        std::vector<std::shared_ptr<Tnode>> _pose_traj;
        // no copy constructor, because a particle should not be explicitly copied
        Particle() = default;
        // create shallow copy of the members
        Particle(const Particle& other) = default;
        // copy assignment. You might be wondering about memory allocation of the vector. They are not 
        // a concern because when calling copy constructor, the old values are properly dropped first.
        // This is different from python, where a variable is simply an alias of an object
        Particle& operator=(const Particle&other) = default;
        // move ctor: the default one moves the members properly
        Particle(Particle&& other) = default;
        // move assignment
        Particle& operator=(Particle&& other) = default;
        // Calls default destructor of all members, and decreases reference count of the shared ptr
        ~Particle() = default; 
    };

    ```
- How to efficiently create a vector of unique ptrs
    ```
    // wrong, this will call copy ctor
    std::vector<T>(n, std::make_unique<>(*another_ptr))
    // right: call this
    push 1 at a time 
    ```
- how to measure memory allocated to the program
    - `getrusage(RUSAGE_SELF, &usage);`
    - 'ru_maxrss' = "maximum resident size", meaning the maximum amount of CPU the process so far has used.
- Do I need move semantics when returning a local object? no. c++11 + already does that for you
- Does cpp immediately return memory when a vector is cleared? No. This is being managed by cpp runtime
    ```
    auto p_list = get_list_of_particles(p_ptr);
    mem_usage_middle = get_memory_usage();
    p_list.clear();
    p_list = get_list_of_particles(p_ptr);
    ```
    - `ru_maxrss` reports stack memory which may not be immediately returned to the OS by cpp
    - However, the second call will reuse the memory
- Does `p_list.clear()`, `p_list.reserve()` do anything as below? 
    ```
    p_list.clear();
    p_list.reserve(1000);
    auto p_list = get_list_of_particles(p_ptr);
    mem_usage_middle = get_memory_usage();
    p_list = get_list_of_particles(p_ptr);
    ```
    - Yes. without `clear()`, it was observed that extra memory is required to create the new list p_ptr. Then the old vector's entries are destroyed. However, this increases the cpp memory stack. So, **it's a good idea to call vector.clear() before assigning it to something else**
    - `p_list.reserve()` also makes sure no extra memory is allocated while vector is appended to. We are having a higher likelihood that the program reuses the previosly reserved memory, which will cap the memory usage of the program


- How to add an error message in gtest? 
    ```
    ASSERT_EQ(mem_usages[i], mem_usages[i-1]) << "Memory usage mismatch at trial " << i; 
    ```

### ROS:
- Can I source a file in launch file? No.

### TODO:
Icp using Least Squares
    1. Data association: nearest data
    2. Compute rotation using SVD
    3. Repeat the above process because data association becomes better
    - Point to plane icp is a good starting point. How does it work?

### Future Improvements
1. Laser scan point association. Currently, we are using point-point association. That is not as robust as point-plane assocation
    - Also, outlier association could be more intricate
2. Information Filter
