/**
 * RoboSight 3D ROS Connection Interface
 * Uses roslibjs to communicate with ROS 1 or ROS 2 via rosbridge_suite websocket.
 */
export class ROSConnector {
    constructor(onStateChange, onFrameReceived) {
        this.ros = null;
        this.isConnected = false;
        this.onStateChange = onStateChange; // (status, message)
        this.onFrameReceived = onFrameReceived; // callback for parsed sensor data
        
        // Topic instances
        this.topics = {
            imu: null,
            rgb: null,
            depth: null,
            detections: null
        };
        
        // Depth map dimensions (will be read from topic, or fallback)
        this.depthWidth = 160;
        this.depthHeight = 120;
    }

    connect(url, topicNames) {
        if (this.isConnected) {
            this.disconnect();
        }

        this.onStateChange('connecting', 'Attempting WebSocket handshake...');
        
        try {
            this.ros = new ROSLIB.Ros({
                url: url
            });

            this.ros.on('connection', () => {
                this.isConnected = true;
                this.onStateChange('connected', 'Successfully connected to ROS bridge.');
                this.subscribeToTopics(topicNames);
            });

            this.ros.on('error', (error) => {
                this.isConnected = false;
                this.onStateChange('error', 'ROS Connection error. Check bridge server.');
                console.error('ROS error:', error);
            });

            this.ros.on('close', () => {
                this.isConnected = false;
                this.onStateChange('disconnected', 'ROS Connection closed.');
                this.unsubscribeAll();
            });

        } catch (err) {
            this.isConnected = false;
            this.onStateChange('error', 'Invalid WebSocket URL or client configuration.');
            console.error('ROS bridge init error:', err);
        }
    }

    disconnect() {
        if (this.ros) {
            this.unsubscribeAll();
            this.ros.close();
            this.ros = null;
        }
        this.isConnected = false;
        this.onStateChange('disconnected', 'ROS connection terminated.');
    }

    subscribeToTopics(topicNames) {
        if (!this.ros || !this.isConnected) return;
        
        this.unsubscribeAll();
        
        // Setup cmd_vel publisher
        this.cmdVelTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        });
        
        // Helper to subscribe to IMU
        this.topics.imu = new ROSLIB.Topic({
            ros: this.ros,
            name: topicNames.imu,
            messageType: 'sensor_msgs/Imu'
        });
        this.topics.imu.subscribe(msg => this.handleIMU(msg));

        // Helper to subscribe to RGB Camera (handles compressed)
        const isCompressed = topicNames.rgb.includes('compressed');
        this.topics.rgb = new ROSLIB.Topic({
            ros: this.ros,
            name: topicNames.rgb,
            messageType: isCompressed ? 'sensor_msgs/CompressedImage' : 'sensor_msgs/Image'
        });
        this.topics.rgb.subscribe(msg => this.handleRGB(msg, isCompressed));

        // Helper to subscribe to Depth map
        this.topics.depth = new ROSLIB.Topic({
            ros: this.ros,
            name: topicNames.depth,
            messageType: 'sensor_msgs/Image'
        });
        this.topics.depth.subscribe(msg => this.handleDepth(msg));

        // Helper to subscribe to Detections
        // Supports vision_msgs/Detection2DArray or detection structures
        this.topics.detections = new ROSLIB.Topic({
            ros: this.ros,
            name: topicNames.detect,
            messageType: 'vision_msgs/Detection2DArray'
        });
        this.topics.detections.subscribe(msg => this.handleDetections(msg));
    }

    unsubscribeAll() {
        Object.keys(this.topics).forEach(key => {
            if (this.topics[key]) {
                this.topics[key].unsubscribe();
                this.topics[key] = null;
            }
        });
        if (this.cmdVelTopic) {
            this.cmdVelTopic = null;
        }
    }

    /* Message Handlers and Translators */

    handleIMU(msg) {
        // Parse quaternion to Euler (Roll, Pitch, Yaw)
        const q = msg.orientation;
        
        // Quaternion to Euler formula
        const roll = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
        const pitch = Math.asin(2 * (q.w * q.y - q.z * q.x));
        const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        
        const imuData = {
            attitude: {
                roll: roll * (180 / Math.PI),
                pitch: pitch * (180 / Math.PI),
                yaw: yaw * (180 / Math.PI)
            },
            linearAccel: [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            angularVel: [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        };

        this.onFrameReceived('imu', imuData);
    }

    handleRGB(msg, isCompressed) {
        if (isCompressed) {
            // Compressed image comes as JPEG/PNG base64
            const img = new Image();
            img.src = `data:image/jpeg;base64,${msg.data}`;
            img.onload = () => {
                this.onFrameReceived('image', img);
            };
        } else {
            // Raw image parsing (from sensor_msgs/Image)
            // msg.data is unit8 array, we create canvas data
            const canvas = document.createElement('canvas');
            canvas.width = msg.width;
            canvas.height = msg.height;
            const ctx = canvas.getContext('2d');
            const imgData = ctx.createImageData(msg.width, msg.height);
            
            // Fill image (ROS is usually RGB/BGR, canvas needs RGBA)
            const isBGR = msg.encoding.includes('bgr');
            for (let i = 0, j = 0; i < msg.data.length; i += 3, j += 4) {
                imgData.data[j]   = isBGR ? msg.data[i+2] : msg.data[i];     // Red
                imgData.data[j+1] = msg.data[i+1];                           // Green
                imgData.data[j+2] = isBGR ? msg.data[i]   : msg.data[i+2];   // Blue
                imgData.data[j+3] = 255;                                     // Alpha
            }
            ctx.putImageData(imgData, 0, 0);
            this.onFrameReceived('image', canvas);
        }
    }

    handleDepth(msg) {
        // ROS depth images can be 32FC1 (meters float32) or 16UC1 (millimeters uint16)
        this.depthWidth = msg.width;
        this.depthHeight = msg.height;
        
        let depthFloats = null;
        
        // Parse raw depth data
        if (msg.encoding === '32FC1') {
            // Float32 array
            // Base64 decode string data into float array
            const binary = atob(msg.data);
            const bytes = new Uint8Array(binary.length);
            for (let i = 0; i < binary.length; i++) {
                bytes[i] = binary.charCodeAt(i);
            }
            depthFloats = new Float32Array(bytes.buffer);
        } else if (msg.encoding === '16UC1' || msg.encoding.includes('16U')) {
            // Uint16 array in millimeters, convert to meters
            const binary = atob(msg.data);
            const bytes = new Uint8Array(binary.length);
            for (let i = 0; i < binary.length; i++) {
                bytes[i] = binary.charCodeAt(i);
            }
            const raw16 = new Uint16Array(bytes.buffer);
            depthFloats = new Float32Array(raw16.length);
            for (let i = 0; i < raw16.length; i++) {
                depthFloats[i] = raw16[i] / 1000.0; // mm to meters
            }
        }
        
        if (depthFloats) {
            this.onFrameReceived('depth', {
                depth: depthFloats,
                width: this.depthWidth,
                height: this.depthHeight
            });
        }
    }

    handleDetections(msg) {
        // Parse vision_msgs/Detection2DArray
        // We extract bounding boxes and project them
        const parsedDetections = msg.detections.map(d => {
            const bbox = d.bbox;
            const x = bbox.center.x;
            const y = bbox.center.y;
            const w = bbox.size_x;
            const h = bbox.size_y;
            
            // Class identification from results
            const score = d.results[0] ? d.results[0].score : 1.0;
            const classIdStr = d.results[0] ? String(d.results[0].id) : 'object';
            
            // Map common classes
            let className = 'object';
            if (classIdStr.toLowerCase().includes('person') || classIdStr === '0') {
                className = 'person';
            }
            
            return {
                class: className,
                confidence: score,
                // BBox as [minU, minV, maxU, maxV]
                bbox2d: [x - w/2, y - h/2, x + w/2, y + h/2],
                
                // Set default 3D dimensions relative to sensor
                // These will be refined by our spatial pipeline using the point cloud depth
                centroid: [0, 0, 0],
                size: [0.5, 0.5, 0.5]
            };
        });

        this.onFrameReceived('detections', parsedDetections);
    }

    publishCmdVel(linearX, angularZ) {
        if (!this.ros || !this.isConnected || !this.cmdVelTopic) return;
        
        const twist = new ROSLIB.Message({
            linear: {
                x: linearX,
                y: 0.0,
                z: 0.0
            },
            angular: {
                x: 0.0,
                y: 0.0,
                z: angularZ
            }
        });
        
        this.cmdVelTopic.publish(twist);
    }
}
