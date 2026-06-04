/**
 * RoboSight 3D Map Reconstruction (SLAM) Engine
 * Transforms local sensor coordinates to global coordinates and builds a cumulative 3D map.
 */
export class MapReconstruction {
    constructor() {
        this.trajectory = [];      // Array of global robot coordinates: [{x, y, z}, ...]
        this.globalPoints = [];     // Accumulated static point cloud: [x0, y0, z0, x1, y1, z1, ...]
        this.globalColors = [];     // Accumulated static colors: [r0, g0, b0, ...]
        
        // Sampling control
        this.lastMapUpdatePose = { x: 0, y: 0, z: 0 };
        this.updateDistanceThreshold = 0.4; // Add a keyframe point cloud every 40cm traveled
        this.maxMapPoints = 120000;         // Cap global points to prevent WebGL overhead
        
        // Keep track of stationary mapped objects
        this.mappedObjects = new Map();     // key: entity ID, value: centroid/size
    }

    reset() {
        this.trajectory = [];
        this.globalPoints = [];
        this.globalColors = [];
        this.lastMapUpdatePose = { x: 0, y: 0, z: 0 };
        this.mappedObjects.clear();
    }

    /**
     * Updates SLAM trajectory and fusions local point cloud into global map if robot moved significantly
     */
    update(robotPose, localPoints, localColors, localGridMap, depthData, W, H, intrinsics) {
        // 1. Record current pose in trajectory
        const currentPos = { x: robotPose.x, y: robotPose.y, z: robotPose.z };
        
        // Only append to trajectory if moved slightly or empty
        if (this.trajectory.length === 0) {
            this.trajectory.push({ ...currentPos });
        } else {
            const last = this.trajectory[this.trajectory.length - 1];
            const dist = Math.sqrt(Math.pow(currentPos.x - last.x, 2) + Math.pow(currentPos.y - last.y, 2) + Math.pow(currentPos.z - last.z, 2));
            if (dist > 0.05) {
                this.trajectory.push({ ...currentPos });
                // Keep history capped for trajectory path line
                if (this.trajectory.length > 500) this.trajectory.shift();
            }
        }

        // 2. Determine if we should perform keyframe point fusion (avoid doing this every frame for performance)
        const distFromLastKeyframe = Math.sqrt(
            Math.pow(currentPos.x - this.lastMapUpdatePose.x, 2) +
            Math.pow(currentPos.y - this.lastMapUpdatePose.y, 2) +
            Math.pow(currentPos.z - this.lastMapUpdatePose.z, 2)
        );

        const isFirstFrame = this.globalPoints.length === 0;
        
        if (isFirstFrame || distFromLastKeyframe > this.updateDistanceThreshold) {
            this.lastMapUpdatePose = { ...currentPos };
            this.fuseKeyframe(robotPose, localPoints, localColors);
        }

        return {
            trajectory: this.trajectory,
            globalPointsCount: this.globalPoints.length / 3,
            mappedObjectsCount: this.mappedObjects.size
        };
    }

    /**
     * Transforms and fusions local camera point cloud coordinates into the global map frame
     */
    fuseKeyframe(pose, localPoints, localColors) {
        if (!localPoints || localPoints.length === 0) return;

        // Build Transformation Matrix from Robot Pose (yaw, pitch, roll, position)
        // Camera axes are: X right, Y up, Z forward (out of camera is +Z, but in local frame we mapped -Z into screen)
        // Euler rotations:
        const rollRad = pose.roll;
        const pitchRad = pose.pitch;
        const yawRad = pose.yaw;

        const translation = new THREE.Vector3(pose.x, pose.y, pose.z);
        const rotationEuler = new THREE.Euler(pitchRad, yawRad, rollRad, 'YXZ');
        
        const transformMatrix = new THREE.Matrix4();
        transformMatrix.makeRotationFromEuler(rotationEuler);
        transformMatrix.setPosition(translation);

        // Downsample local cloud during fusion to conserve memory
        // e.g. sample 1 out of every 6 points
        const step = 8;
        const count = localPoints.length / 3;
        
        const tempVec = new THREE.Vector3();

        for (let i = 0; i < count; i += step) {
            const idx = i * 3;
            const lx = localPoints[idx];
            const ly = localPoints[idx + 1];
            const lz = localPoints[idx + 2]; // this is already negative in Three coordinates

            // Apply 3D coordinate rotation and translation
            tempVec.set(lx, ly, lz);
            tempVec.applyMatrix4(transformMatrix);

            this.globalPoints.push(tempVec.x, tempVec.y, tempVec.z);
            this.globalColors.push(localColors[idx], localColors[idx+1], localColors[idx+2]);
        }

        // Limit the total size of the static background point cloud
        if (this.globalPoints.length > this.maxMapPoints * 3) {
            // Remove oldest points
            const pointsToRemove = this.globalPoints.length - this.maxMapPoints * 3;
            this.globalPoints.splice(0, pointsToRemove);
            this.globalColors.splice(0, pointsToRemove);
        }
    }

    /**
     * Traverses detections to accumulate stationary mapped object coordinates in world space.
     */
    accumulateObjects(detections, pose) {
        const rollRad = pose.roll;
        const pitchRad = pose.pitch;
        const yawRad = pose.yaw;

        const translation = new THREE.Vector3(pose.x, pose.y, pose.z);
        const rotationEuler = new THREE.Euler(pitchRad, yawRad, rollRad, 'YXZ');
        
        const transformMatrix = new THREE.Matrix4();
        transformMatrix.makeRotationFromEuler(rotationEuler);
        transformMatrix.setPosition(translation);
        
        const tempVec = new THREE.Vector3();

        detections.forEach((det, idx) => {
            // We only map stationary objects (like cargo boxes/chairs) to prevent trails for moving persons
            if (det.class !== 'object') return;
            
            // Generate world centroid for object
            const cx = det.centroid[0];
            const cy = det.centroid[1];
            const cz = det.centroid[2]; // local relative -z coordinate in Three viewport
            
            tempVec.set(cx, cy, cz);
            tempVec.applyMatrix4(transformMatrix);
            
            const key = `static_object_${idx}`; // Simple index keying for demo
            
            this.mappedObjects.set(key, {
                class: det.class,
                centroid: [tempVec.x, tempVec.y, tempVec.z],
                size: [...det.size],
                yaw: yawRad + det.rotation[2] // combine yaw
            });
        });
    }

    getGlobalCloud() {
        return {
            positions: new Float32Array(this.globalPoints),
            colors: new Float32Array(this.globalColors)
        };
    }
}
