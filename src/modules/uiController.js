/**
 * RoboSight 3D UI & Visualization Controller
 * Orchestrates Three.js rendering, Chart.js graphs, canvas overlays, and HUD updates.
 */
export class UIController {
    constructor() {
        // Viewport dimensions
        this.camCanvas = document.getElementById('canvas-camera');
        this.camCtx = this.camCanvas.getContext('2d');
        
        // Three.js instances
        this.threeContainer = document.getElementById('three-container');
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        
        // Three.js scene meshes
        this.localPointsMesh = null;
        this.localMeshMesh = null;
        this.globalPointsMesh = null;
        this.robotMesh = null;
        this.trajectoryLine = null;
        this.bboxGroup = null;
        
        // Settings
        this.showGrid = true;
        this.gridHelper = null;
        this.meshMode = 'wireframe';
        
        // Chart.js instances
        this.timelineChart = null;
        
        // UI DOM links
        this.dom = {
            f1Val: document.getElementById('val-f1'),
            f1Path: document.getElementById('gauge-f1-path'),
            precisionVal: document.getElementById('val-precision'),
            recallVal: document.getElementById('val-recall'),
            iouVal: document.getElementById('val-iou'),
            iouPath: document.getElementById('gauge-iou-path'),
            iou2dVal: document.getElementById('val-iou-2d'),
            distVal: document.getElementById('val-distance-error'),
            distBar: document.getElementById('val-distance-bar'),
            jitterVal: document.getElementById('val-jitter'),
            latencyCounter: document.getElementById('latency-counter'),
            fpsCounter: document.getElementById('fps-counter'),
            detectionCount: document.getElementById('detection-count'),
            pointsCount: document.getElementById('voxels-count'),
            
            // Scraping / Collision Proximity HUD
            scrapeDot: document.getElementById('scrape-indicator-dot'),
            scrapeText: document.getElementById('scrape-indicator-text'),
            scrapeCard: document.getElementById('scrape-metric-card'),
            scrapeCount: document.getElementById('val-scrape-count'),
            scrapeDist: document.getElementById('val-scrape-dist'),
            scrapeStatus: document.getElementById('scrape-hud-status'),
            
            // IMU Dials
            rollNeedle: document.getElementById('needle-roll'),
            pitchNeedle: document.getElementById('needle-pitch'),
            yawNeedle: document.getElementById('needle-yaw'),
            rollVal: document.getElementById('val-roll'),
            pitchVal: document.getElementById('val-pitch'),
            yawVal: document.getElementById('val-yaw'),
            accelVal: document.getElementById('val-accel'),
            gyroVal: document.getElementById('val-gyro'),
            poseVal: document.getElementById('val-pose'),
            
            // Pipeline Nodes
            nodeTopics: document.getElementById('node-topics'),
            nodeCloud: document.getElementById('node-cloud'),
            nodeMesh: document.getElementById('node-mesh'),
            nodeReproj: document.getElementById('node-reprojection'),
            nodeMap: document.getElementById('node-map'),
            
            // Pipeline Rates
            topicsRate: document.getElementById('node-topics-rate'),
            cloudRate: document.getElementById('node-cloud-rate'),
            meshRate: document.getElementById('node-mesh-rate'),
            reprojRate: document.getElementById('node-reproj-rate'),
            mapRate: document.getElementById('node-map-rate'),
            
            // Arrows
            arrow1: document.getElementById('arrow-1'),
            arrow2: document.getElementById('arrow-2'),
            arrow3: document.getElementById('arrow-3'),
            arrow4: document.getElementById('arrow-4'),
            
            // Gallery
            galleryContainer: document.getElementById('gallery-container'),
            galleryEmpty: document.getElementById('gallery-empty')
        };
        
        // Initialize graphs and canvas dimensions
        this.initThree();
        this.initCharts();
        this.resizeCanvases();
        
        window.addEventListener('resize', () => {
            this.resizeCanvases();
            this.resizeThree();
        });
    }

    /**
     * Initializes the Three.js WebGL rendering environment
     */
    initThree() {
        const loadingEl = document.getElementById('three-loading');
        if (loadingEl) loadingEl.style.display = 'flex';

        // 1. Create Scene & Renderer
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x080b11);
        
        // Fog for depth perception
        this.scene.fog = new THREE.FogExp2(0x080b11, 0.08);

        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(this.threeContainer.clientWidth, this.threeContainer.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;
        this.threeContainer.appendChild(this.renderer.domElement);

        // 2. Camera Setup
        this.camera = new THREE.PerspectiveCamera(
            55,
            this.threeContainer.clientWidth / this.threeContainer.clientHeight,
            0.1,
            50.0
        );
        // Position camera behind and looking down at origin
        this.camera.position.set(3, 3, 5);

        // 3. Orbit Controls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.maxDistance = 15.0;
        this.controls.minDistance = 0.5;
        this.controls.target.set(0, 0, -2); // point controls at target space
        
        // 4. Ground Grid & Coordinate Axis Guides
        this.gridHelper = new THREE.GridHelper(20, 20, 0x24314E, 0x161D30);
        this.gridHelper.position.y = -1.0; // aligns with corridor floor
        this.scene.add(this.gridHelper);
        
        const axesHelper = new THREE.AxesHelper(1.0);
        axesHelper.position.set(0, -0.99, 0);
        this.scene.add(axesHelper);

        // 5. Lighting (Mesh shading needs lights)
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.45);
        this.scene.add(ambientLight);

        const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
        dirLight.position.set(5, 8, 2);
        this.scene.add(dirLight);

        // 6. Point Cloud Material & Mesh Setup
        // Local point cloud
        const pointsGeom = new THREE.BufferGeometry();
        const pointsMat = new THREE.PointsMaterial({
            size: 0.04,
            vertexColors: true,
            transparent: true,
            opacity: 0.85
        });
        this.localPointsMesh = new THREE.Points(pointsGeom, pointsMat);
        this.scene.add(this.localPointsMesh);

        // Global SLAM point cloud
        const globalGeom = new THREE.BufferGeometry();
        const globalMat = new THREE.PointsMaterial({
            size: 0.03,
            vertexColors: true,
            transparent: true,
            opacity: 0.4 // lower opacity to distinguish from active scan
        });
        this.globalPointsMesh = new THREE.Points(globalGeom, globalMat);
        this.scene.add(this.globalPointsMesh);

        // Reconstructed mesh model
        const meshGeom = new THREE.BufferGeometry();
        const meshMat = new THREE.MeshStandardMaterial({
            roughness: 0.7,
            metalness: 0.1,
            side: THREE.DoubleSide
        });
        this.localMeshMesh = new THREE.Mesh(meshGeom, meshMat);
        this.scene.add(this.localMeshMesh);

        // 7. Group container for dynamic 3D Bounding Boxes
        this.bboxGroup = new THREE.Group();
        this.scene.add(this.bboxGroup);

        // 8. Visual Robot Model (Simple cylinder with direction cone)
        this.robotMesh = new THREE.Group();
        
        const chassisGeom = new THREE.CylinderGeometry(0.25, 0.25, 0.15, 16);
        const chassisMat = new THREE.MeshStandardMaterial({ color: 0x00a8ff, roughness: 0.4 });
        const chassis = new THREE.Mesh(chassisGeom, chassisMat);
        this.robotMesh.add(chassis);

        const noseGeom = new THREE.ConeGeometry(0.1, 0.25, 8);
        noseGeom.rotateX(Math.PI / 2); // point cone forward along Z
        noseGeom.translate(0, 0, -0.2); // offset to nose edge
        const noseMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff });
        const nose = new THREE.Mesh(noseGeom, noseMat);
        this.robotMesh.add(nose);
        
        // Add glowing purple UVC disinfection cylinder
        const uvcGeom = new THREE.CylinderGeometry(0.24, 0.26, 0.8, 16);
        uvcGeom.translate(0, -0.4, 0); // translate down relative to chassis center
        const uvcMat = new THREE.MeshBasicMaterial({
            color: 0x9b5de5,
            transparent: true,
            opacity: 0.35,
            side: THREE.DoubleSide
        });
        this.uvcLight = new THREE.Mesh(uvcGeom, uvcMat);
        this.robotMesh.add(this.uvcLight);
        this.uvcLight.visible = false;
        
        this.scene.add(this.robotMesh);

        // 9. Robot Path Trajectory Line
        const trajGeom = new THREE.BufferGeometry();
        const trajMat = new THREE.LineBasicMaterial({ color: 0x00f0ff, linewidth: 2 });
        this.trajectoryLine = new THREE.Line(trajGeom, trajMat);
        this.scene.add(this.trajectoryLine);

        if (loadingEl) loadingEl.style.display = 'none';
        this.animateThree();
    }

    /**
     * WebGL frame rendering loop
     */
    animateThree() {
        requestAnimationFrame(() => this.animateThree());
        
        if (this.controls) {
            this.controls.update();
        }
        
        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
        }
    }

    resetThreeCamera() {
        this.camera.position.set(3, 3, 5);
        this.controls.target.set(0, 0, -2);
    }

    toggleGrid() {
        this.showGrid = !this.showGrid;
        this.gridHelper.visible = this.showGrid;
    }

    resizeThree() {
        if (!this.renderer) return;
        this.camera.aspect = this.threeContainer.clientWidth / this.threeContainer.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.threeContainer.clientWidth, this.threeContainer.clientHeight);
    }

    resizeCanvases() {
        // Ensure 2D canvas displays at crisp bounds
        const parent = this.camCanvas.parentElement;
        this.camCanvas.width = parent.clientWidth;
        this.camCanvas.height = parent.clientHeight;
    }

    /**
     * Initializes the metrics Chart.js timeline
     */
    initCharts() {
        const ctx = document.getElementById('chart-timeline').getContext('2d');
        
        const labels = Array.from({ length: 40 }, (_, i) => i - 40);
        
        this.timelineChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: labels,
                datasets: [
                    {
                        label: 'F1 Score',
                        data: Array(40).fill(0),
                        borderColor: '#34C759',
                        backgroundColor: 'rgba(52, 199, 89, 0.05)',
                        borderWidth: 2,
                        tension: 0.2,
                        yAxisID: 'y'
                    },
                    {
                        label: '3D IoU',
                        data: Array(40).fill(0),
                        borderColor: '#00F0FF',
                        backgroundColor: 'transparent',
                        borderWidth: 2,
                        borderDash: [4, 4],
                        tension: 0.2,
                        yAxisID: 'y'
                    },
                    {
                        label: 'Localization Error (cm)',
                        data: Array(40).fill(0),
                        borderColor: '#FF9F0A',
                        backgroundColor: 'transparent',
                        borderWidth: 2,
                        tension: 0.2,
                        yAxisID: 'yError'
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    x: {
                        grid: { color: 'rgba(94, 111, 136, 0.1)' },
                        ticks: { color: '#90a1b9', font: { size: 9 } }
                    },
                    y: {
                        position: 'left',
                        min: 0,
                        max: 1.0,
                        grid: { color: 'rgba(94, 111, 136, 0.1)' },
                        ticks: { color: '#90a1b9', font: { size: 9 } },
                        title: { display: true, text: 'Accuracy Rate', color: '#90a1b9', font: { size: 9 } }
                    },
                    yError: {
                        position: 'right',
                        min: 0,
                        max: 100, // cm
                        grid: { drawOnChartArea: false },
                        ticks: { color: '#FF9F0A', font: { size: 9 } },
                        title: { display: true, text: 'Centroid error (cm)', color: '#FF9F0A', font: { size: 9 } }
                    }
                },
                plugins: {
                    legend: {
                        labels: { color: '#f0f4f9', boxWidth: 10, font: { size: 10 } },
                        position: 'top'
                    }
                }
            }
        });
    }

    /**
     * Appends metrics snapshot to the timeline graph data stream
     */
    updateChart(metrics) {
        if (!this.timelineChart) return;
        
        const data = this.timelineChart.data.datasets;
        
        // Shift values
        data[0].data.shift();
        data[0].data.push(metrics.f1Score);
        
        data[1].data.shift();
        data[1].data.push(metrics.meanIou3D);
        
        data[2].data.shift();
        // Convert distance error to centimeters
        data[2].data.push(metrics.meanDistanceError * 100);
        
        this.timelineChart.update('none'); // silent update
    }

    /**
     * Redraws 2D canvas frame including original RGB, 2D BBoxes, and projected 3D wireframe boxes
     */
    update2DViewport(imgSrc, groundTruth, detections, pipeline, intrinsics) {
        const ctx = this.camCtx;
        const w = this.camCanvas.width;
        const h = this.camCanvas.height;
        
        ctx.clearRect(0, 0, w, h);
        
        // Draw primary camera feed
        if (imgSrc) {
            ctx.drawImage(imgSrc, 0, 0, w, h);
        } else {
            ctx.fillStyle = '#05070a';
            ctx.fillRect(0, 0, w, h);
        }

        // Draw Ground Truth bounding boxes (dashed magenta lines)
        ctx.save();
        groundTruth.forEach(gt => {
            const bbox = gt.bbox2d;
            // Map bbox resolution (320x240) to UI canvas dimensions
            const scaleX = w / 320;
            const scaleY = h / 240;
            
            const bx = bbox[0] * scaleX;
            const by = bbox[1] * scaleY;
            const bw = (bbox[2] - bbox[0]) * scaleX;
            const bh = (bbox[3] - bbox[1]) * scaleY;

            ctx.strokeStyle = '#BD00FF'; // Ground Truth Magenta
            ctx.lineWidth = 1.5;
            ctx.setLineDash([4, 4]);
            ctx.strokeRect(bx, by, bw, bh);
            
            // Small tag label
            ctx.fillStyle = '#BD00FF';
            ctx.font = '9px monospace';
            ctx.fillText(`GT: ${gt.class}`, bx, Math.max(10, by - 4));
        });
        ctx.restore();

        // Draw Predicted Detections and reprojected 3D wireframes (solid cyan/green lines)
        detections.forEach(det => {
            const bbox = det.bbox2d;
            const scaleX = w / 320;
            const scaleY = h / 240;
            
            const bx = bbox[0] * scaleX;
            const by = bbox[1] * scaleY;
            const bw = (bbox[2] - bbox[0]) * scaleX;
            const bh = (bbox[3] - bbox[1]) * scaleY;

            // Draw 2D box
            ctx.strokeStyle = '#00F0FF'; // Cyber Cyan
            ctx.lineWidth = 2;
            ctx.strokeRect(bx, by, bw, bh);
            
            // Draw Confidence value
            ctx.fillStyle = '#00F0FF';
            ctx.font = 'bold 9px monospace';
            ctx.fillText(`${det.class} (${(det.confidence * 100).toFixed(0)}%)`, bx + 2, by + 10);

            // Calculate and draw reprojected 3D Bbox wireframe on canvas
            if (det.centroid && det.size) {
                // Project 3D box corners
                const corners = pipeline.reproject3DBoxTo2D(
                    det.centroid,
                    det.size,
                    det.rotation ? det.rotation[2] : 0,
                    intrinsics,
                    w,
                    h
                );

                this.drawProjected3DBox(ctx, corners, w, h);
            }
        });
    }

    /**
     * Renders a 3D wireframe box projected on the 2D canvas
     */
    drawProjected3DBox(ctx, corners, w, h) {
        if (!corners || corners.length < 8) return;
        
        // Map corners to UI scale
        const scaledCorners = corners.map(c => {
            if (!c) return null;
            // Map 320x240 coordinates to viewport width/height
            return {
                x: c.u * (w / 320),
                y: c.v * (h / 240)
            };
        });

        // Skip drawing if any corner was behind camera
        if (scaledCorners.includes(null)) return;

        ctx.save();
        ctx.strokeStyle = '#34C759'; // Neon Green for projected 3D matches
        ctx.lineWidth = 1.5;
        
        // Draw bottom face edges [0, 1, 2, 3]
        ctx.beginPath();
        ctx.moveTo(scaledCorners[0].x, scaledCorners[0].y);
        ctx.lineTo(scaledCorners[1].x, scaledCorners[1].y);
        ctx.lineTo(scaledCorners[2].x, scaledCorners[2].y);
        ctx.lineTo(scaledCorners[3].x, scaledCorners[3].y);
        ctx.closePath();
        ctx.stroke();

        // Draw top face edges [4, 5, 6, 7]
        ctx.beginPath();
        ctx.moveTo(scaledCorners[4].x, scaledCorners[4].y);
        ctx.lineTo(scaledCorners[5].x, scaledCorners[5].y);
        ctx.lineTo(scaledCorners[6].x, scaledCorners[6].y);
        ctx.lineTo(scaledCorners[7].x, scaledCorners[7].y);
        ctx.closePath();
        ctx.stroke();

        // Draw connecting vertical pillars [0-4, 1-5, 2-6, 3-7]
        for (let i = 0; i < 4; i++) {
            ctx.beginPath();
            ctx.moveTo(scaledCorners[i].x, scaledCorners[i].y);
            ctx.lineTo(scaledCorners[i+4].x, scaledCorners[i+4].y);
            ctx.stroke();
        }
        ctx.restore();
    }

    /**
     * Updates Three.js WebGL scenes (local point cloud, mesh, trajectory, map)
     */
    update3DScene(pointCloudData, meshIndices, depthData, W, H, robotPose, SLAMMap, meshMode, activeMode) {
        this.meshMode = meshMode;
        
        // 1. Update Robot position and rotation in 3D scene
        this.robotMesh.position.set(robotPose.x, robotPose.y, -robotPose.z);
        this.robotMesh.rotation.set(robotPose.pitch, -robotPose.yaw, robotPose.roll);

        if (this.uvcLight) {
            this.uvcLight.visible = (activeMode === 'disinfect');
        }

        // 2. Update local Point Cloud
        if (pointCloudData && pointCloudData.positions.length > 0) {
            const positions = pointCloudData.positions;
            const colors = pointCloudData.colors;
            
            // Re-allocate or populate points geometries
            if (this.localPointsMesh.geometry) this.localPointsMesh.geometry.dispose();
            
            const geom = new THREE.BufferGeometry();
            geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
            
            this.localPointsMesh.geometry = geom;
            this.localPointsMesh.visible = (meshMode === 'none' || meshMode === 'wireframe');
        }

        // 3. Update local 3D Surface Mesh
        if (meshIndices && meshIndices.length > 0 && meshMode !== 'none') {
            const positions = pointCloudData.positions;
            
            if (this.localMeshMesh.geometry) this.localMeshMesh.geometry.dispose();
            
            const geom = new THREE.BufferGeometry();
            geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            geom.setIndex(new THREE.BufferAttribute(meshIndices, 1));
            geom.computeVertexNormals();
            
            this.localMeshMesh.geometry = geom;
            this.localMeshMesh.material.wireframe = (meshMode === 'wireframe');
            
            // Give subtle neon wireframe color
            this.localMeshMesh.material.color.setHex(meshMode === 'wireframe' ? 0x263757 : 0x161d30);
            this.localMeshMesh.visible = true;
        } else {
            this.localMeshMesh.visible = false;
        }

        // 4. Update Global SLAM map points
        const globalCloud = SLAMMap.getGlobalCloud();
        if (globalCloud && globalCloud.positions.length > 0) {
            if (this.globalPointsMesh.geometry) this.globalPointsMesh.geometry.dispose();
            
            const geom = new THREE.BufferGeometry();
            geom.setAttribute('position', new THREE.BufferAttribute(globalCloud.positions, 3));
            geom.setAttribute('color', new THREE.BufferAttribute(globalCloud.colors, 3));
            
            this.globalPointsMesh.geometry = geom;
        }

        // 5. Update Robot Trajectory Line
        if (SLAMMap.trajectory.length > 1) {
            const pathPoints = [];
            SLAMMap.trajectory.forEach(pt => {
                pathPoints.push(new THREE.Vector3(pt.x, pt.y, -pt.z));
            });
            
            if (this.trajectoryLine.geometry) this.trajectoryLine.geometry.dispose();
            
            const geom = new THREE.BufferGeometry().setFromPoints(pathPoints);
            this.trajectoryLine.geometry = geom;
        }
    }

    /**
     * Redraws 3D wireframe boxes in Three.js viewport representing Ground Truth and Detections
     */
    update3DBboxes(detections, groundTruth) {
        // Clear previous box drawings
        while (this.bboxGroup.children.length > 0) {
            const child = this.bboxGroup.children[0];
            if (child.geometry) child.geometry.dispose();
            if (child.material) child.material.dispose();
            this.bboxGroup.remove(child);
        }

        // Renders GT boxes in purple/pink
        groundTruth.forEach(gt => {
            const centroid = gt.centroid;
            const size = gt.size;
            
            // Create wireframe box
            const geom = new THREE.BoxGeometry(size[0], size[1], size[2]);
            const mat = new THREE.MeshBasicMaterial({
                color: 0xbd00ff,
                wireframe: true,
                transparent: true,
                opacity: 0.45
            });
            const mesh = new THREE.Mesh(geom, mat);
            
            // local frame projection maps -z
            mesh.position.set(centroid[0], centroid[1], -centroid[2]);
            
            // Add to dynamic bbox overlay group
            this.bboxGroup.add(mesh);
        });

        // Renders predictions in cyan/green
        detections.forEach(det => {
            const centroid = det.centroid;
            const size = det.size;
            const rot = det.rotation;

            const geom = new THREE.BoxGeometry(size[0], size[1], size[2]);
            const mat = new THREE.MeshBasicMaterial({
                color: 0x00f0ff,
                wireframe: true,
                transparent: true,
                opacity: 0.8
            });
            const mesh = new THREE.Mesh(geom, mat);
            mesh.position.set(centroid[0], centroid[1], -centroid[2]);
            if (rot) {
                mesh.rotation.set(rot[0], -rot[2], rot[1]); // orient aligned
            }
            
            this.bboxGroup.add(mesh);
        });
    }

    /**
     * Updates dashboard numerical values, status labels, gauges, compass needles, and pipeline flow
     */
    updateHUD(metrics, imu, pose, frameRate, parsedCloud, SLAMInfo, isSimActive, topicsHz, scrapeInfo, sessionScrapesCount, sessionMinScrapeDistance) {
        // 1. Text Metrics Updates
        this.dom.f1Val.innerText = metrics.f1Score.toFixed(2);
        this.dom.precisionVal.innerText = metrics.precision.toFixed(2);
        this.dom.recallVal.innerText = metrics.recall.toFixed(2);
        this.dom.iouVal.innerText = `${Math.round(metrics.meanIou3D * 100)}%`;
        this.dom.iou2dVal.innerText = `${Math.round(metrics.meanIou2D * 100)}%`;
        this.dom.distVal.innerText = metrics.meanDistanceError.toFixed(2);
        this.dom.jitterVal.innerText = `±${metrics.jitter.toFixed(2)}m`;
        this.dom.fpsCounter.innerText = Math.round(frameRate);
        this.dom.latencyCounter.innerText = `${metrics.latency.toFixed(0)} ms`;
        
        // 2. Metrics Gauge Rings Dash-offset update
        // Stroke circumference is 2 * pi * 15.9155 = 100.
        // Thus, strokeDasharray="85, 100" means 85% full.
        const f1Percent = Math.round(metrics.f1Score * 100);
        this.dom.f1Path.setAttribute('stroke-dasharray', `${f1Percent}, 100`);
        
        const iouPercent = Math.round(metrics.meanIou3D * 100);
        this.dom.iouPath.setAttribute('stroke-dasharray', `${iouPercent}, 100`);

        // Distance error indicator bar (maps up to 0.5 meters error)
        const errorPercent = Math.min(100, (metrics.meanDistanceError / 0.5) * 100);
        this.dom.distBar.style.width = `${errorPercent}%`;

        // 3. Counter Badges
        this.dom.detectionCount.innerText = `${metrics.matchedCount + metrics.falseCount} DETECTIONS`;
        this.dom.pointsCount.innerText = `${parsedCloud.vertexCount} POINTS`;

        // 4. IMU Telemetry COMPASS Needle Rotations
        // Yaw needle is compass dial (rotates inverse to body heading)
        this.dom.yawNeedle.style.transform = `rotate(${-imu.attitude.yaw}deg)`;
        this.dom.yawVal.innerText = `${imu.attitude.yaw.toFixed(1)}°`;

        this.dom.rollNeedle.style.transform = `rotate(${imu.attitude.roll}deg)`;
        this.dom.rollVal.innerText = `${imu.attitude.roll.toFixed(1)}°`;

        this.dom.pitchNeedle.style.transform = `rotate(${-imu.attitude.pitch}deg)`;
        this.dom.pitchVal.innerText = `${imu.attitude.pitch.toFixed(1)}°`;

        // Accel, Gyro & Pose strings
        this.dom.accelVal.innerText = imu.linearAccel.map(v => v.toFixed(2)).join(', ') + ' m/s²';
        this.dom.gyroVal.innerText = imu.angularVel.map(v => v.toFixed(2)).join(', ') + ' rad/s';
        this.dom.poseVal.innerText = `X: ${pose.x.toFixed(2)}m, Y: ${pose.y.toFixed(2)}m, Z: ${pose.z.toFixed(2)}m`;

        // 5. Real-Time Pipeline Flow Node highlighting
        this.updatePipelineNodes(parsedCloud, SLAMInfo, frameRate, isSimActive, topicsHz);

        // 6. Bot Scraping / Proximity warning indicators
        if (scrapeInfo && this.dom.scrapeDot) {
            const isAlarm = scrapeInfo.isScraping;
            
            // Indicator Dot
            this.dom.scrapeDot.className = isAlarm ? 'status-dot pulse-red animate-flash' : 'status-dot pulse-green';
            
            // Indicator Text
            this.dom.scrapeText.className = isAlarm ? 'status-value text-red animate-flash' : 'status-value text-green';
            this.dom.scrapeText.innerText = isAlarm ? 'ALARM' : 'SAFE';
            
            // Metric Card alarm class
            if (isAlarm) {
                this.dom.scrapeCard.classList.add('alarm');
                this.dom.scrapeStatus.innerHTML = `Status: <span class="text-red animate-flash" style="font-weight: bold;">ALARM</span>`;
                this.dom.scrapeCount.className = 'error-num text-red';
            } else {
                this.dom.scrapeCard.classList.remove('alarm');
                this.dom.scrapeStatus.innerHTML = `Status: <span class="text-green" style="font-weight: bold;">SAFE</span>`;
                this.dom.scrapeCount.className = 'error-num text-green';
            }
            
            // Scrape count & minimum distance
            this.dom.scrapeCount.innerText = sessionScrapesCount;
            this.dom.scrapeDist.innerText = (sessionMinScrapeDistance === 99.0 ? '0.00' : sessionMinScrapeDistance.toFixed(2)) + 'm';
        }
    }

    /**
     * Computes updates for the pipeline flow visualizer states
     */
    updatePipelineNodes(parsedCloud, SLAMInfo, frameRate, isSimActive, topicsHz) {
        const hzStr = isSimActive ? `${topicsHz} Hz` : (topicsHz > 0 ? `${topicsHz} Hz` : 'Offline');
        
        // Node 1: Topics
        this.dom.topicsRate.innerText = hzStr;
        this.toggleNodeActive(this.dom.nodeTopics, topicsHz > 0);
        this.toggleArrowActive(this.dom.arrow1, topicsHz > 0);

        // Node 2: Point Cloud
        const cloudActive = (topicsHz > 0 && parsedCloud.vertexCount > 0);
        this.dom.cloudRate.innerText = cloudActive ? `${Math.round(frameRate)} Hz` : 'Idle';
        this.toggleNodeActive(this.dom.nodeCloud, cloudActive);
        this.toggleArrowActive(this.dom.arrow2, cloudActive);

        // Node 3: Mesh
        const meshActive = (cloudActive && this.meshMode !== 'none');
        this.dom.meshRate.innerText = meshActive ? `${Math.round(frameRate)} Hz` : 'Disabled';
        this.toggleNodeActive(this.dom.nodeMesh, meshActive);
        this.toggleArrowActive(this.dom.arrow3, meshActive);

        // Node 4: Reprojection
        this.dom.reprojRate.innerText = cloudActive ? `${Math.round(frameRate)} Hz` : 'Idle';
        this.toggleNodeActive(this.dom.nodeReproj, cloudActive);
        this.toggleArrowActive(this.dom.arrow4, cloudActive);

        // Node 5: SLAM Map
        const mapActive = (cloudActive && SLAMInfo.globalPointsCount > 0);
        this.dom.mapRate.innerText = mapActive ? `${SLAMInfo.globalPointsCount} pts` : 'Idle';
        this.toggleNodeActive(this.dom.nodeMap, mapActive);
    }

    toggleNodeActive(el, active) {
        if (active) {
            el.classList.add('active');
        } else {
            el.classList.remove('active');
        }
    }

    toggleArrowActive(el, active) {
        if (active) {
            el.classList.add('active');
        } else {
            el.classList.remove('active');
        }
    }

    /**
     * Adds a visual card for a false detection capture.
     */
    addCaptureCard(capture, onRemove, onDownload, onDownloadJSON) {
        // Hide empty state if visible
        if (this.dom.galleryEmpty) {
            this.dom.galleryEmpty.style.display = 'none';
        }

        // Create card wrapper
        const card = document.createElement('div');
        card.className = 'capture-card';
        card.id = `cap-card-${capture.id}`;

        const isFP = capture.type === 'FP';
        const typeClass = isFP ? 'type-fp' : 'type-fn';
        const typeLabel = isFP ? 'FALSE POSITIVE' : 'FALSE NEGATIVE';

        card.innerHTML = `
            <img class="capture-thumbnail" src="${capture.image}" alt="Detection Error">
            <div class="capture-info">
                <span class="capture-type ${typeClass}">${typeLabel}</span>
                <span class="capture-details" title="${capture.details}">${capture.details}</span>
                <span class="capture-time">${capture.timeStr}</span>
                <div class="capture-actions">
                    <button class="btn-mini btn-dl" title="Download Image">PNG</button>
                    <button class="btn-mini btn-json" title="Export JSON">JSON</button>
                    <button class="btn-mini btn-del" style="background: rgba(255,59,48,0.15); border-color: rgba(255,59,48,0.25); color: var(--accent-red);" title="Delete Log">DEL</button>
                </div>
            </div>
        `;

        // Bind inner actions
        card.querySelector('.btn-dl').addEventListener('click', () => onDownload(capture));
        card.querySelector('.btn-json').addEventListener('click', () => onDownloadJSON(capture));
        card.querySelector('.btn-del').addEventListener('click', () => {
            card.remove();
            onRemove(capture.id);
            
            // If gallery is empty, show empty state
            const childCards = this.dom.galleryContainer.querySelectorAll('.capture-card');
            if (childCards.length === 0) {
                this.dom.galleryEmpty.style.display = 'flex';
            }
        });

        // Insert at the beginning (newest first)
        this.dom.galleryContainer.insertBefore(card, this.dom.galleryContainer.firstChild);
    }

    clearGallery() {
        // Clear all cards
        const container = this.dom.galleryContainer;
        const cards = container.querySelectorAll('.capture-card');
        cards.forEach(card => card.remove());
        
        // Restore empty state
        if (this.dom.galleryEmpty) {
            this.dom.galleryEmpty.style.display = 'flex';
        }
    }
}
