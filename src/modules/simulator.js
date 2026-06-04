/**
 * RoboSight 3D Simulation Engine
 * Generates realistic real-time sensor streams (RGB, Depth, IMU) and detections.
 */
export class RoboticsSimulator {
    constructor() {
        this.isRunning = false;
        this.scenario = 'office';
        this.speed = 1.0;
        this.noiseEnabled = true;
        this.dropoutsEnabled = false;
        this.fpsDropEnabled = false;
        
        // Target canvas resolution for visual camera feed
        this.camWidth = 320;
        this.camHeight = 240;
        
        // Depth map resolution (default medium: 160x120)
        this.depthWidth = 160;
        this.depthHeight = 120;
        
        // Simulation time variable
        this.time = 0;
        
        // Camera Intrinsic Defaults (matches index.html inputs)
        this.intrinsics = {
            fx: 190, // scaled for depth map resolution
            fy: 190,
            cx: 80,
            cy: 60
        };

        // Create HTML5 offscreen canvases for visual generation
        this.rgbCanvas = document.createElement('canvas');
        this.rgbCanvas.width = this.camWidth;
        this.rgbCanvas.height = this.camHeight;
        this.ctx = this.rgbCanvas.getContext('2d');
        
        // Robot State Variables
        this.robotPose = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
        this.linearVelocity = { x: 0, y: 0, z: 0 };
        this.angularVelocity = { x: 0, y: 0, z: 0 };
        
        // World Entities
        this.entities = [
            {
                id: 'person_0',
                class: 'person',
                color: '#FF3B30', // Red for person
                centroid: { x: 0, y: 0, z: 3.5 }, // 3D pos relative to robot
                size: { w: 0.6, h: 1.7, d: 0.5 }, // 3D dimensions (meters)
                speed: 0.5,
                phase: 0
            },
            {
                id: 'box_0',
                class: 'object',
                color: '#FF9F0A', // Orange for object (cargo box)
                centroid: { x: 1.2, y: -0.4, z: 2.8 },
                size: { w: 0.7, h: 0.7, d: 0.7 },
                speed: 0,
                phase: Math.PI / 4
            }
        ];
    }

    start() {
        this.isRunning = true;
    }

    stop() {
        this.isRunning = false;
    }

    reset() {
        this.time = 0;
        this.robotPose = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
        this.entities[0].centroid = { x: 0, y: 0, z: 3.5 };
        this.entities[1].centroid = { x: 1.2, y: -0.4, z: 2.8 };
    }

    setParams(density, depthCutoff) {
        if (density === 'low') {
            this.depthWidth = 80;
            this.depthHeight = 60;
        } else if (density === 'high') {
            this.depthWidth = 320;
            this.depthHeight = 240;
        } else {
            this.depthWidth = 160;
            this.depthHeight = 120;
        }
        
        // Scale intrinsics according to depth resolution
        const scaleX = this.depthWidth / 320;
        const scaleY = this.depthHeight / 240;
        this.intrinsics.fx = 380 * scaleX;
        this.intrinsics.fy = 380 * scaleY;
        this.intrinsics.cx = 160 * scaleX;
        this.intrinsics.cy = 120 * scaleY;
    }

    /**
     * Ticks the simulation forward and generates a frame of data
     * @param {number} deltaTimeSeconds 
     */
    step(deltaTimeSeconds) {
        if (!this.isRunning) return null;
        
        const dt = deltaTimeSeconds * this.speed;
        this.time += dt;
        
        // 1. Update Robot Odometry (Lissajous sway movement to simulate walking/rolling)
        this.robotPose.x = Math.sin(this.time * 0.4) * 0.8;
        this.robotPose.z = (this.time * 0.2); // Continuous forward travel
        
        // Add subtle rotational sway from ground bumps
        this.robotPose.roll = Math.sin(this.time * 2.5) * 0.02; // in radians
        this.robotPose.pitch = Math.cos(this.time * 3.0) * 0.015;
        this.robotPose.yaw = Math.sin(this.time * 0.5) * 0.15 + (Math.sin(this.time * 1.5) * 0.01);
        
        // Calculate velocity vectors
        this.linearVelocity.x = Math.cos(this.time * 0.4) * 0.32;
        this.linearVelocity.z = 0.2;
        this.linearVelocity.y = 0;
        
        this.angularVelocity.x = Math.cos(this.time * 2.5) * 0.05;
        this.angularVelocity.y = Math.cos(this.time * 0.5) * 0.075;
        this.angularVelocity.z = 0;

        // 2. Update Entities (simulated people moving)
        // Person walking back and forth sideways in front of robot
        const person = this.entities[0];
        person.centroid.x = Math.sin(this.time * 0.8) * 1.2;
        // Keep distance around 3-4 meters
        person.centroid.z = 3.6 + Math.cos(this.time * 0.4) * 0.6;
        
        // Box is stationary relative to map, sway its relative pos based on robot pose
        const box = this.entities[1];
        // Centroid in local frame moves as robot moves
        
        // 3. Render Visual RGB Camera Feed
        this.renderRGBFeed();
        
        // 4. Generate Depth Map Array (aligned with depth grid)
        const depthData = this.generateDepthMap();
        
        // 5. Generate 2D/3D Bounding Boxes
        const groundTruth = this.generateGroundTruth();
        const detections = this.generateDetections(groundTruth);
        
        // 6. Generate IMU Telemetry
        const imu = this.generateIMUTelemetry(dt);
        
        return {
            timestamp: this.time,
            imu: imu,
            pose: { ...this.robotPose },
            image: this.rgbCanvas,
            depth: depthData,
            depthWidth: this.depthWidth,
            depthHeight: this.depthHeight,
            groundTruth: groundTruth,
            detections: detections,
            intrinsics: { ...this.intrinsics }
        };
    }

    /**
     * Renders a simulated corridor view onto the RGB Canvas
     */
    renderRGBFeed() {
        const ctx = this.ctx;
        const w = this.camWidth;
        const h = this.camHeight;
        
        // Background - Dark Corridor perspective
        ctx.fillStyle = '#0a0d16';
        ctx.fillRect(0, 0, w, h);
        
        // Draw floor perspective
        ctx.fillStyle = '#111726';
        ctx.beginPath();
        ctx.moveTo(0, h);
        ctx.lineTo(w * 0.2, h * 0.5);
        ctx.lineTo(w * 0.8, h * 0.5);
        ctx.lineTo(w, h);
        ctx.closePath();
        ctx.fill();
        
        // Draw perspective walls
        ctx.fillStyle = '#171f33';
        // Left wall
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(w * 0.2, h * 0.5);
        ctx.lineTo(0, h);
        ctx.closePath();
        ctx.fill();
        // Right wall
        ctx.beginPath();
        ctx.moveTo(w, 0);
        ctx.lineTo(w * 0.8, h * 0.5);
        ctx.lineTo(w, h);
        ctx.closePath();
        ctx.fill();
        
        // Ceiling
        ctx.fillStyle = '#080a11';
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(w * 0.2, h * 0.5);
        ctx.lineTo(w * 0.8, h * 0.5);
        ctx.lineTo(w, 0);
        ctx.closePath();
        ctx.fill();

        // Draw structural vertical pillar stripes to make point cloud look structured
        ctx.strokeStyle = '#222d4a';
        ctx.lineWidth = 2;
        for (let i = 0.2; i <= 0.8; i += 0.2) {
            ctx.beginPath();
            ctx.moveTo(w * i, h * 0.5);
            ctx.lineTo(w * (i < 0.5 ? i - 0.2 : i + 0.2), h);
            ctx.stroke();
        }

        // Draw entities projected onto the camera viewport
        this.entities.forEach(ent => {
            const relX = ent.centroid.x;
            const relY = ent.centroid.y;
            const relZ = ent.centroid.z;
            
            // Project 3D center to 2D screen coordinates using camera properties
            // Screen scale uses camera parameters (320x240 image resolution)
            const fx2d = 380;
            const fy2d = 380;
            const cx2d = 160;
            const cy2d = 120;
            
            // Local projection
            const projX = (relX * fx2d) / relZ + cx2d;
            // Coordinate frame inversion: Y is up in 3D, down in canvas
            const projY = (-relY * fy2d) / relZ + cy2d;
            
            // Calculate screen size based on distance Z
            const sizeH = (ent.size.h * fy2d) / relZ;
            const sizeW = (ent.size.w * fx2d) / relZ;
            
            const screenX = projX - sizeW / 2;
            const screenY = projY - sizeH / 2;
            
            if (relZ > 0.5) { // Check frustum plane
                ctx.save();
                ctx.fillStyle = ent.color;
                ctx.shadowColor = ent.color;
                ctx.shadowBlur = 10;
                
                // Draw simulated entity bounding boxes
                if (ent.class === 'person') {
                    // Draw person pill shape
                    ctx.beginPath();
                    this.roundRect(ctx, screenX, screenY, sizeW, sizeH, sizeW * 0.4);
                    ctx.fill();
                    
                    // Draw a quick visor/head detail
                    ctx.fillStyle = '#ffffff';
                    ctx.beginPath();
                    ctx.arc(projX, screenY + sizeH * 0.15, sizeW * 0.2, 0, Math.PI * 2);
                    ctx.fill();
                } else {
                    // Draw static cargo box
                    ctx.fillRect(screenX, screenY, sizeW, sizeH);
                    
                    // Draw crossed strap lines on crate
                    ctx.strokeStyle = '#000000';
                    ctx.lineWidth = Math.max(1, 3 - relZ * 0.4);
                    ctx.strokeRect(screenX + 2, screenY + 2, sizeW - 4, sizeH - 4);
                }
                ctx.restore();
            }
        });
    }

    /**
     * Helper to draw a rounded rectangle
     */
    roundRect(ctx, x, y, width, height, radius) {
        ctx.beginPath();
        ctx.moveTo(x + radius, y);
        ctx.lineTo(x + width - radius, y);
        ctx.quadraticCurveTo(x + width, y, x + width, y + radius);
        ctx.lineTo(x + width, y + height - radius);
        ctx.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
        ctx.lineTo(x + radius, y + height);
        ctx.quadraticCurveTo(x, y + height, x, y + height - radius);
        ctx.lineTo(x, y + radius);
        ctx.quadraticCurveTo(x, y, x + radius, y);
        ctx.closePath();
    }

    /**
     * Generates a structural depth map matching a 3D corridor
     */
    generateDepthMap() {
        const W = this.depthWidth;
        const H = this.depthHeight;
        const depth = new Float32Array(W * H);
        
        const fx = this.intrinsics.fx;
        const fy = this.intrinsics.fy;
        const cx = this.intrinsics.cx;
        const cy = this.intrinsics.cy;
        
        // Define corridor parameters
        const floorY = -1.0;  // Floor height relative to camera
        const ceilY = 1.0;    // Ceiling height relative to camera
        const wallLeftX = -1.5; // Left wall X
        const wallRightX = 1.5; // Right wall X
        const endWallZ = 7.0;   // Background corridor end wall Z
        
        for (let v = 0; v < H; v++) {
            for (let u = 0; u < W; u++) {
                const idx = v * W + u;
                
                // Direction ray vectors from intrinsics
                const rayX = (u - cx) / fx;
                // Note: v coordinate increases downwards, so flip for vertical 3D axis
                const rayY = -(v - cy) / fy;
                
                let minZ = endWallZ; // default end of corridor
                
                // Floor plane intersection
                if (rayY < -0.01) {
                    const zFloor = floorY / rayY;
                    if (zFloor > 0 && zFloor < minZ) minZ = zFloor;
                }
                
                // Ceiling plane intersection
                if (rayY > 0.01) {
                    const zCeil = ceilY / rayY;
                    if (zCeil > 0 && zCeil < minZ) minZ = zCeil;
                }
                
                // Left Wall plane intersection
                if (rayX < -0.01) {
                    const zLeft = wallLeftX / rayX;
                    if (zLeft > 0 && zLeft < minZ) minZ = zLeft;
                }
                
                // Right Wall plane intersection
                if (rayX > 0.01) {
                    const zRight = wallRightX / rayX;
                    if (zRight > 0 && zRight < minZ) minZ = zRight;
                }
                
                // Ray tracing overlapping entities in the depth buffer
                this.entities.forEach(ent => {
                    const eX = ent.centroid.x;
                    const eY = ent.centroid.y;
                    const eZ = ent.centroid.z;
                    
                    const wHalf = ent.size.w / 2;
                    const hHalf = ent.size.h / 2;
                    const dHalf = ent.size.d / 2;
                    
                    // Fast bounding box intersection test along camera ray
                    // Target box range in 3D: [eX - wHalf, eX + wHalf], [eY - hHalf, eY + hHalf], [eZ - dHalf, eZ + dHalf]
                    // Point on ray: P(z) = [rayX * z, rayY * z, z]
                    
                    // Solve for z when ray intersects box faces
                    // For Z range: [eZ - dHalf, eZ + dHalf]
                    const zNear = eZ - dHalf;
                    const zFar = eZ + dHalf;
                    
                    // Check intersection of ray at zNear
                    const intersectX = rayX * zNear;
                    const intersectY = rayY * zNear;
                    
                    if (intersectX >= eX - wHalf && intersectX <= eX + wHalf &&
                        intersectY >= eY - hHalf && intersectY <= eY + hHalf) {
                        if (zNear > 0.1 && zNear < minZ) {
                            minZ = zNear;
                        }
                    }
                });
                
                // Add sensor noise (standard RGB-D camera noise is proportional to Z^2)
                if (this.noiseEnabled) {
                    const noiseSigma = 0.01 + 0.005 * minZ * minZ;
                    // Box-Muller transform for gaussian noise
                    const u1 = Math.random();
                    const u2 = Math.random();
                    const gNoise = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
                    minZ += gNoise * noiseSigma;
                }
                
                depth[idx] = minZ;
            }
        }
        return depth;
    }

    /**
     * Generates Ground Truth 3D and 2D bounding boxes in camera frame coordinates
     */
    generateGroundTruth() {
        const fx = 380;
        const fy = 380;
        const cx = 160;
        const cy = 120;
        
        return this.entities.map(ent => {
            const x = ent.centroid.x;
            const y = ent.centroid.y;
            const z = ent.centroid.z;
            
            const w = ent.size.w;
            const h = ent.size.h;
            const d = ent.size.d;
            
            // Calculate 2D Box projecting 8 corners and taking min/max
            const corners = [
                { x: x - w/2, y: y - h/2, z: z - d/2 },
                { x: x + w/2, y: y - h/2, z: z - d/2 },
                { x: x - w/2, y: y + h/2, z: z - d/2 },
                { x: x + w/2, y: y + h/2, z: z - d/2 },
                { x: x - w/2, y: y - h/2, z: z + d/2 },
                { x: x + w/2, y: y - h/2, z: z + d/2 },
                { x: x - w/2, y: y + h/2, z: z + d/2 },
                { x: x + w/2, y: y + h/2, z: z + d/2 }
            ];
            
            let minU = Infinity, maxU = -Infinity;
            let minV = Infinity, maxV = -Infinity;
            
            corners.forEach(c => {
                if (c.z > 0.1) {
                    const u = (c.x * fx) / c.z + cx;
                    const v = (-c.y * fy) / c.z + cy; // flip Y
                    minU = Math.min(minU, u);
                    maxU = Math.max(maxU, u);
                    minV = Math.min(minV, v);
                    maxV = Math.max(maxV, v);
                }
            });
            
            // Clip to screen boundaries
            minU = Math.max(0, Math.min(this.camWidth, minU));
            maxU = Math.max(0, Math.min(this.camWidth, maxU));
            minV = Math.max(0, Math.min(this.camHeight, minV));
            maxV = Math.max(0, Math.min(this.camHeight, maxV));

            return {
                id: ent.id,
                class: ent.class,
                centroid: [x, y, z],
                size: [w, h, d],
                rotation: [0, 0, 0], // pitch, roll, yaw
                bbox2d: [minU, minV, maxU, maxV]
            };
        });
    }

    /**
     * Generates simulated detections with noise and omissions
     */
    generateDetections(groundTruth) {
        const detections = [];
        
        groundTruth.forEach(gt => {
            // Check dropout setting
            if (this.dropoutsEnabled && gt.class === 'person' && Math.random() < 0.15) {
                // Drop this detection (simulated false negative)
                return;
            }
            
            let centroid = [...gt.centroid];
            let size = [...gt.size];
            let bbox2d = [...gt.bbox2d];
            
            // Inject spatial jitter
            if (this.noiseEnabled) {
                centroid[0] += (Math.random() - 0.5) * 0.10; // 10cm drift
                centroid[1] += (Math.random() - 0.5) * 0.05;
                centroid[2] += (Math.random() - 0.5) * 0.12;
                
                size[0] += (Math.random() - 0.5) * 0.08;
                size[1] += (Math.random() - 0.5) * 0.08;
                size[2] += (Math.random() - 0.5) * 0.08;
                
                bbox2d[0] += (Math.random() - 0.5) * 5; // pixel jitter
                bbox2d[1] += (Math.random() - 0.5) * 5;
                bbox2d[2] += (Math.random() - 0.5) * 5;
                bbox2d[3] += (Math.random() - 0.5) * 5;
            }
            
            detections.push({
                class: gt.class,
                centroid: centroid,
                size: size,
                rotation: [0, 0, (Math.random() - 0.5) * 0.05], // slight yaw drift
                bbox2d: bbox2d,
                confidence: 0.75 + Math.random() * 0.22
            });
        });
        
        // Add occasional False Positive
        if (Math.random() < 0.05) {
            detections.push({
                class: Math.random() > 0.5 ? 'person' : 'object',
                centroid: [-1.2 + Math.random() * 2.4, -0.4, 1.5 + Math.random() * 3.5],
                size: [0.5, 0.8, 0.5],
                rotation: [0, 0, 0],
                bbox2d: [10 + Math.random()*200, 30 + Math.random()*100, 80 + Math.random()*200, 150 + Math.random()*80],
                confidence: 0.61
            });
        }
        
        return detections;
    }

    /**
     * Compiles mock IMU metrics
     */
    generateIMUTelemetry(dt) {
        // Roll, pitch, yaw from robot state
        const r = this.robotPose.roll;
        const p = this.robotPose.pitch;
        const y = this.robotPose.yaw;
        
        // Accelerometer reads gravity (9.81 m/s^2) in Z, plus linear acceleration
        // In local body frame, gravity gets rotated by pitch and roll
        const gLocalX = -Math.sin(p) * 9.81;
        const gLocalY = Math.sin(r) * Math.cos(p) * 9.81;
        const gLocalZ = Math.cos(r) * Math.cos(p) * 9.81;
        
        return {
            attitude: {
                roll: r * (180 / Math.PI), // deg
                pitch: p * (180 / Math.PI),
                yaw: y * (180 / Math.PI)
            },
            linearAccel: [
                this.linearVelocity.x * 0.2 + gLocalX,
                this.linearVelocity.y * 0.2 + gLocalY,
                this.linearVelocity.z * 0.1 + gLocalZ
            ],
            angularVel: [
                this.angularVelocity.x,
                this.angularVelocity.y,
                this.angularVelocity.z
            ]
        };
    }
}
