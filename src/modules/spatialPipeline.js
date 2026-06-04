/**
 * RoboSight 3D Spatial Processing Pipeline
 * Math core for 3D coordinate unprojection, mesh construction, 3D BBox estimation, and reprojection.
 */
export class SpatialPipeline {
    constructor() {
        this.depthCutoff = 4.5; // max meter range
        this.discontinuityThreshold = 0.35; // mesh edge threshold (meters)
    }

    /**
     * Converts a 2D depth map array into a 3D point cloud
     * @param {Float32Array} depthData 
     * @param {number} W width of depth map
     * @param {number} H height of depth map
     * @param {object} intrinsics Camera intrinsics
     * @param {HTMLCanvasElement} rgbCanvas Reference canvas for texturing
     */
    unprojectPointCloud(depthData, W, H, intrinsics, rgbCanvas) {
        const fx = intrinsics.fx;
        const fy = intrinsics.fy;
        const cx = intrinsics.cx;
        const cy = intrinsics.cy;

        // Sample RGB canvas image data for coloring points
        let rgbData = null;
        if (rgbCanvas) {
            const ctx = rgbCanvas.getContext('2d');
            try {
                rgbData = ctx.getImageData(0, 0, rgbCanvas.width, rgbCanvas.height).data;
            } catch (e) {
                // Cross-origin fallback (if video is cross-origin)
            }
        }
        
        const rgbW = rgbCanvas ? rgbCanvas.width : 1;
        const rgbH = rgbCanvas ? rgbCanvas.height : 1;

        // Prepare typed arrays for Three.js geometry buffers
        // We will output: positions [x, y, z] and colors [r, g, b]
        const positions = [];
        const colors = [];
        
        // We'll also store an index mapping to build the mesh
        // mapping grid index to active point index in the vertex buffer
        const gridToVertexIdx = new Int32Array(W * H).fill(-1);
        let activeIdx = 0;

        for (let v = 0; v < H; v++) {
            for (let u = 0; u < W; u++) {
                const gridIdx = v * W + u;
                const d = depthData[gridIdx];

                // Skip points outside range or invalid
                if (isNaN(d) || d <= 0.1 || d > this.depthCutoff) {
                    continue;
                }

                // 3D coordinate unprojection math
                // X = (u - cx) * Z / fx
                // Y = -(v - cy) * Z / fy  (invert Y since image coordinates grow downwards)
                // Z = Z
                const x = ((u - cx) * d) / fx;
                const y = -((v - cy) * d) / fy;
                const z = d;

                positions.push(x, y, -z); // Three.js uses RHS: -z goes into the screen

                // Map pixel to RGB source
                let r = 0.5, g = 0.5, b = 0.8; // default cyber-blue color
                if (rgbData) {
                    // Map depth resolution (e.g. 160x120) to image resolution (e.g. 320x240)
                    const uRGB = Math.floor((u / W) * rgbW);
                    const vRGB = Math.floor((v / H) * rgbH);
                    const imgIdx = (vRGB * rgbW + uRGB) * 4;
                    
                    if (imgIdx < rgbData.length) {
                        r = rgbData[imgIdx] / 255;
                        g = rgbData[imgIdx+1] / 255;
                        b = rgbData[imgIdx+2] / 255;
                    }
                } else {
                    // Height-based custom gradient fallback
                    r = 0.1 + (y + 1) * 0.4;
                    g = 0.4 + (d / 8) * 0.5;
                    b = 0.9 - (y + 1) * 0.3;
                }

                colors.push(r, g, b);
                gridToVertexIdx[gridIdx] = activeIdx;
                activeIdx++;
            }
        }

        return {
            positions: new Float32Array(positions),
            colors: new Float32Array(colors),
            gridMap: gridToVertexIdx,
            vertexCount: activeIdx
        };
    }

    /**
     * Connects point cloud vertices into a triangulated 3D Mesh
     * Prevents stretching across depth discontinuities
     */
    generateMeshIndices(gridMap, depthData, W, H) {
        const indices = [];

        for (let r = 0; r < H - 1; r++) {
            for (let c = 0; c < W - 1; c++) {
                // Four corners of current cell
                const i00 = r * W + c;
                const i10 = r * W + (c + 1);
                const i01 = (r + 1) * W + c;
                const i11 = (r + 1) * W + (c + 1);

                // Get active vertex index from grid map
                const v00 = gridMap[i00];
                const v10 = gridMap[i10];
                const v01 = gridMap[i01];
                const v11 = gridMap[i11];

                // Skip if any corner point was skipped in point cloud
                if (v00 < 0 || v10 < 0 || v01 < 0 || v11 < 0) continue;

                // Get depth values for discontinuity checks
                const d00 = depthData[i00];
                const d10 = depthData[i10];
                const d01 = depthData[i01];
                const d11 = depthData[i11];

                // Triangle 1: [v00, v01, v10] (lower left half)
                // Only create triangle if depths are close to each other
                const diffT1_1 = Math.abs(d00 - d01);
                const diffT1_2 = Math.abs(d00 - d10);
                const diffT1_3 = Math.abs(d01 - d10);
                if (diffT1_1 < this.discontinuityThreshold && 
                    diffT1_2 < this.discontinuityThreshold && 
                    diffT1_3 < this.discontinuityThreshold) {
                    indices.push(v00, v01, v10);
                }

                // Triangle 2: [v10, v01, v11] (upper right half)
                const diffT2_1 = Math.abs(d11 - d10);
                const diffT2_2 = Math.abs(d11 - d01);
                const diffT2_3 = Math.abs(d10 - d01);
                if (diffT2_1 < this.discontinuityThreshold && 
                    diffT2_2 < this.discontinuityThreshold && 
                    diffT2_3 < this.discontinuityThreshold) {
                    indices.push(v10, v01, v11);
                }
            }
        }

        return new Uint32Array(indices);
    }

    /**
     * Estimates 3D Bounding Box centroids and sizes from 2D camera boxes
     * by querying point cloud vertices inside the 2D bounding boxes.
     */
    estimate3DBboxes(detections2d, depthData, W, H, intrinsics) {
        const fx = intrinsics.fx;
        const fy = intrinsics.fy;
        const cx = intrinsics.cx;
        const cy = intrinsics.cy;

        // Scale ratio between depth map and detection resolution (which is RGB size 320x240)
        // detections2d boxes are in 320x240 coordinate system
        const scaleU = W / 320;
        const scaleV = H / 240;

        return detections2d.map(det => {
            const bbox = det.bbox2d;
            
            // Map 2D box boundaries to depth coordinates
            const uMin = Math.floor(bbox[0] * scaleU);
            const vMin = Math.floor(bbox[1] * scaleV);
            const uMax = Math.ceil(bbox[2] * scaleU);
            const vMax = Math.ceil(bbox[3] * scaleV);

            // Collect all depth values inside 2D box region
            const depths = [];
            for (let v = Math.max(0, vMin); v < Math.min(H, vMax); v++) {
                for (let u = Math.max(0, uMin); u < Math.min(W, uMax); u++) {
                    const d = depthData[v * W + u];
                    if (d > 0.1 && d < this.depthCutoff) {
                        depths.push({ u, v, d });
                    }
                }
            }

            if (depths.length < 5) {
                // Fail-safe default if point cloud contains no depth pixels in box
                // Project center using arbitrary default distance (e.g. 3.0 meters)
                const centerU = (bbox[0] + bbox[2]) / 2 * scaleU;
                const centerV = (bbox[1] + bbox[3]) / 2 * scaleV;
                const defZ = 3.0;
                const defX = ((centerU - cx) * defZ) / fx;
                const defY = -((centerV - cy) * defZ) / fy;
                return {
                    ...det,
                    centroid: [defX, defY, defZ],
                    size: [0.6, det.class === 'person' ? 1.7 : 0.6, 0.6]
                };
            }

            // Find median depth of pixels inside region (filters out back/foreground outliers)
            depths.sort((a, b) => a.d - b.d);
            const mid = Math.floor(depths.length / 2);
            const medianDepth = depths[mid].d;

            // Keep points within standard box depth range (e.g., +/- 0.5 meters around median)
            const clusterPoints = [];
            let sumX = 0, sumY = 0, sumZ = 0;
            
            depths.forEach(p => {
                if (Math.abs(p.d - medianDepth) < 0.6) {
                    const x = ((p.u - cx) * p.d) / fx;
                    const y = -((p.v - cy) * p.d) / fy;
                    const z = p.d;
                    clusterPoints.push({ x, y, z });
                    sumX += x;
                    sumY += y;
                    sumZ += z;
                }
            });

            if (clusterPoints.length === 0) {
                clusterPoints.push({
                    x: ((depths[mid].u - cx) * medianDepth) / fx,
                    y: -((depths[mid].v - cy) * medianDepth) / fy,
                    z: medianDepth
                });
                sumX = clusterPoints[0].x;
                sumY = clusterPoints[0].y;
                sumZ = clusterPoints[0].z;
            }

            // Calculate 3D centroid (average)
            const numPts = clusterPoints.length;
            const centroid = [sumX / numPts, sumY / numPts, sumZ / numPts];

            // Calculate bounding size dimensions [width, height, depth]
            let minX = Infinity, maxX = -Infinity;
            let minY = Infinity, maxY = -Infinity;
            let minZ = Infinity, maxZ = -Infinity;

            clusterPoints.forEach(p => {
                minX = Math.min(minX, p.x);
                maxX = Math.max(maxX, p.x);
                minY = Math.min(minY, p.y);
                maxY = Math.max(maxY, p.y);
                minZ = Math.min(minZ, p.z);
                maxZ = Math.max(maxZ, p.z);
            });

            // Enforce realistic bounds based on class
            let width = Math.max(0.3, maxX - minX);
            let height = Math.max(0.3, maxY - minY);
            let depth = Math.max(0.3, maxZ - minZ);

            if (det.class === 'person') {
                width = Math.min(1.0, Math.max(0.4, width));
                height = Math.min(2.1, Math.max(1.4, height));
                depth = Math.min(0.8, Math.max(0.3, depth));
            } else {
                width = Math.min(1.5, width);
                height = Math.min(1.5, height);
                depth = Math.min(1.5, depth);
            }

            return {
                ...det,
                centroid: [centroid[0], centroid[1], centroid[2]],
                size: [width, height, depth]
            };
        });
    }

    /**
     * Projects 3D corners of oriented bounding boxes back to 2D image coordinates.
     */
    reproject3DBoxTo2D(centroid, size, rotationYaw, intrinsics, camW, camH) {
        // scale scale scaling matching RGB camera feeds (320x240)
        // scale intrinsics if necessary
        const fx = 380;
        const fy = 380;
        const cx = 160;
        const cy = 120;
        
        const cx3d = centroid[0];
        const cy3d = centroid[1];
        const cz3d = centroid[2];
        
        const dx = size[0] / 2;
        const dy = size[1] / 2;
        const dz = size[2] / 2;
        
        // Define local 8 corners of the box centered around origin
        const localCorners = [
            [-dx, -dy, -dz], [dx, -dy, -dz], [dx, dy, -dz], [-dx, dy, -dz],
            [-dx, -dy, dz],  [dx, -dy, dz],  [dx, dy, dz],  [-dx, dy, dz]
        ];

        const cosY = Math.cos(rotationYaw);
        const sinY = Math.sin(rotationYaw);

        // Map and project corners
        const screenCorners = localCorners.map(corner => {
            // Apply rotation around Y axis (yaw)
            const rx = corner[0] * cosY - corner[2] * sinY;
            const ry = corner[1]; // height
            const rz = corner[0] * sinY + corner[2] * cosY;

            // Translate to camera position
            const wx = rx + cx3d;
            const wy = ry + cy3d;
            const wz = rz + cz3d;

            if (wz <= 0.1) return null; // Behind camera clipping

            // Camera Intrinsic Projection Formulas
            const u = (wx * fx) / wz + cx;
            const v = (-wy * fy) / wz + cy; // Invert Y for screen

            return { u, v };
        });

        return screenCorners;
    }

    /**
     * Stage 2: CV-based Bot Scraping / Proximity Collision Identification
     * Analyzes the active local point cloud to detect if obstacles/walls are within the robot's scraping shell.
     * @param {Float32Array} positions Local point positions [x, y, z] in Three.js coordinate system
     * @returns {object} { isScraping: boolean, minDistance: number, scrapeCount: number }
     */
    identifyBotScraping(positions) {
        if (!positions || positions.length === 0) {
            return { isScraping: false, minDistance: 99.0, scrapeCount: 0 };
        }

        // Robot chassis properties in camera frame:
        // Chassis radius is 0.25m. We define warning threshold at 0.32m (7cm buffer)
        const warnThreshold = 0.32;
        // Floor points are at y <= -0.9m. We only check points above floor to prevent false triggers
        const robotBaseMinY = -0.85;
        const robotBaseMaxY = -0.40;

        let minDistance = 99.0;
        let scrapeCount = 0;

        // Loop through all points (step 3 to read x, y, z)
        for (let i = 0; i < positions.length; i += 3) {
            const x = positions[i];
            const y = positions[i+1];
            const tz = positions[i+2]; // threeJS coordinate is negative Z

            // Check if point is vertically aligned with robot chassis height
            if (y >= robotBaseMinY && y <= robotBaseMaxY) {
                // Calculate horizontal distance from camera/robot center (0,0)
                const dist = Math.sqrt(x * x + tz * tz);
                
                if (dist < minDistance) {
                    minDistance = dist;
                }

                if (dist < warnThreshold) {
                    scrapeCount++;
                }
            }
        }

        // We require at least 8 points to trigger alert (filters sensor noise)
        const isScraping = scrapeCount >= 8;

        return {
            isScraping,
            minDistance,
            scrapeCount
        };
    }
}
