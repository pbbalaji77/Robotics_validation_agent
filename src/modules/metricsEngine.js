/**
 * RoboSight 3D Metrics & Evaluation Engine
 * Computes Precision, Recall, F1-score, 2D/3D IoU, distance drift, and latency.
 */
export class MetricsEngine {
    constructor() {
        this.iouThreshold2D = 0.45; // Match threshold for 2D bounding boxes
        this.historySize = 100;      // sliding window frame history
        
        // Window historical buffers
        this.history = [];
        
        // Rolling metrics summaries
        this.f1Score = 0;
        this.precision = 0;
        this.recall = 0;
        this.meanIou3D = 0;
        this.meanIou2D = 0;
        this.meanDistanceError = 0;
        this.jitter = 0;
        this.latencyHistory = [];
    }

    reset() {
        this.history = [];
        this.f1Score = 0;
        this.precision = 0;
        this.recall = 0;
        this.meanIou3D = 0;
        this.meanIou2D = 0;
        this.meanDistanceError = 0;
        this.jitter = 0;
        this.latencyHistory = [];
    }

    /**
     * Ticks the metrics engine with a new frame's data and computes summaries
     */
    evaluateFrame(detections, groundTruth, pipelineLatencyMs) {
        // 1. Calculate 2D IoU, 3D IoU and match boxes
        const frameResults = this.matchDetections(detections, groundTruth);
        
        // Add pipeline latency
        frameResults.latency = pipelineLatencyMs;
        this.latencyHistory.push(pipelineLatencyMs);
        if (this.latencyHistory.length > this.historySize) {
            this.latencyHistory.shift();
        }
        
        // 2. Append to rolling window history
        this.history.push(frameResults);
        if (this.history.length > this.historySize) {
            this.history.shift();
        }

        // 3. Recompute aggregate metrics from sliding window
        this.aggregateMetrics();

        return {
            f1Score: this.f1Score,
            precision: this.precision,
            recall: this.recall,
            meanIou3D: this.meanIou3D,
            meanIou2D: this.meanIou2D,
            meanDistanceError: this.meanDistanceError,
            jitter: this.jitter,
            matchedCount: frameResults.tp,
            missedCount: frameResults.fn,
            falseCount: frameResults.fp
        };
    }

    /**
     * Matches predicted detections with ground truth boxes using IoU and Class
     */
    matchDetections(detections, groundTruth) {
        let tp = 0;
        let fp = 0;
        let fn = 0;
        
        const matchedGtIndices = new Set();
        const matches = [];
        
        // Sort detections by confidence to match highest confidence first
        const sortedDets = [...detections].sort((a, b) => b.confidence - a.confidence);

        sortedDets.forEach(det => {
            let maxIou2D = -1;
            let bestGtIdx = -1;
            
            // Find ground truth box with highest overlapping 2D area
            for (let i = 0; i < groundTruth.length; i++) {
                const gt = groundTruth[i];
                
                // Only match same class (person vs person, object vs object)
                if (det.class !== gt.class) continue;
                
                const iou2d = this.calculateIoU2D(det.bbox2d, gt.bbox2d);
                if (iou2d > maxIou2D) {
                    maxIou2D = iou2d;
                    bestGtIdx = i;
                }
            }

            // Verify if matches exceeds IoU threshold
            if (maxIou2D >= this.iouThreshold2D && bestGtIdx !== -1 && !matchedGtIndices.has(bestGtIdx)) {
                tp++;
                matchedGtIndices.add(bestGtIdx);
                
                // Calculate 3D IoU and spatial distance error
                const gt = groundTruth[bestGtIdx];
                const iou3d = this.calculateIoU3D(det.centroid, det.size, gt.centroid, gt.size);
                
                const distErr = Math.sqrt(
                    Math.pow(det.centroid[0] - gt.centroid[0], 2) +
                    Math.pow(det.centroid[1] - gt.centroid[1], 2) +
                    Math.pow(det.centroid[2] - gt.centroid[2], 2)
                );
                
                matches.push({
                    class: det.class,
                    iou2d: maxIou2D,
                    iou3d: iou3d,
                    distanceError: distErr
                });
            } else {
                // False detection (no match)
                fp++;
            }
        });

        // Unmatched ground truths are False Negatives (misses)
        fn = groundTruth.length - matchedGtIndices.size;

        return {
            tp,
            fp,
            fn,
            matches
        };
    }

    /**
     * Aggregates metrics inside the sliding window buffer
     */
    aggregateMetrics() {
        if (this.history.length === 0) return;

        let totalTp = 0;
        let totalFp = 0;
        let totalFn = 0;
        
        let sumIou2D = 0;
        let sumIou3D = 0;
        let matchCount = 0;
        
        const distErrors = [];

        this.history.forEach(frame => {
            totalTp += frame.tp;
            totalFp += frame.fp;
            totalFn += frame.fn;
            
            frame.matches.forEach(m => {
                sumIou2D += m.iou2d;
                sumIou3D += m.iou3d;
                distErrors.push(m.distanceError);
                matchCount++;
            });
        });

        // Precision = TP / (TP + FP)
        this.precision = totalTp + totalFp > 0 ? totalTp / (totalTp + totalFp) : 0;
        
        // Recall = TP / (TP + FN)
        this.recall = totalTp + totalFn > 0 ? totalTp / (totalTp + totalFn) : 0;
        
        // F1-Score = 2 * (P * R) / (P + R)
        this.f1Score = this.precision + this.recall > 0 
            ? (2 * this.precision * this.recall) / (this.precision + this.recall) 
            : 0;
        
        // Mean IoUs
        this.meanIou2D = matchCount > 0 ? sumIou2D / matchCount : 0;
        this.meanIou3D = matchCount > 0 ? sumIou3D / matchCount : 0;

        // Mean distance and Jitter (standard deviation of errors)
        if (distErrors.length > 0) {
            const sumErr = distErrors.reduce((a, b) => a + b, 0);
            this.meanDistanceError = sumErr / distErrors.length;
            
            // Compute sample standard deviation for jitter
            const sqDiffs = distErrors.map(err => Math.pow(err - this.meanDistanceError, 2));
            const variance = sqDiffs.reduce((a, b) => a + b, 0) / distErrors.length;
            this.jitter = Math.sqrt(variance);
        } else {
            this.meanDistanceError = 0;
            this.jitter = 0;
        }
    }

    /**
     * Computes Intersection over Union of two 2D boxes [x1, y1, x2, y2]
     */
    calculateIoU2D(boxA, boxB) {
        const xA = Math.max(boxA[0], boxB[0]);
        const yA = Math.max(boxA[1], boxB[1]);
        const xB = Math.min(boxA[2], boxB[2]);
        const yB = Math.min(boxA[3], boxB[3]);

        const intersectArea = Math.max(0, xB - xA) * Math.max(0, yB - yA);
        
        const areaA = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1]);
        const areaB = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1]);
        
        const unionArea = areaA + areaB - intersectArea;
        
        return unionArea > 0 ? intersectArea / unionArea : 0;
    }

    /**
     * Computes Axis-Aligned Bounding Box (AABB) 3D Intersection over Union
     * Centroid: [x,y,z], Size: [w,h,d]
     */
    calculateIoU3D(centroidA, sizeA, centroidB, sizeB) {
        // Calculate min/max boundaries along each 3D axis
        const minAx = centroidA[0] - sizeA[0]/2;
        const maxAx = centroidA[0] + sizeA[0]/2;
        const minAy = centroidA[1] - sizeA[1]/2;
        const maxAy = centroidA[1] + sizeA[1]/2;
        const minAz = centroidA[2] - sizeA[2]/2;
        const maxAz = centroidA[2] + sizeA[2]/2;

        const minBx = centroidB[0] - sizeB[0]/2;
        const maxBx = centroidB[0] + sizeB[0]/2;
        const minBy = centroidB[1] - sizeB[1]/2;
        const maxBy = centroidB[1] + sizeB[1]/2;
        const minBz = centroidB[2] - sizeB[2]/2;
        const maxBz = centroidB[2] + sizeB[2]/2;

        // Calculate overlapping distances along axes
        const xOverlap = Math.max(0, Math.min(maxAx, maxBx) - Math.max(minAx, minBx));
        const yOverlap = Math.max(0, Math.min(maxAy, maxBy) - Math.max(minAy, minBy));
        const zOverlap = Math.max(0, Math.min(maxAz, maxBz) - Math.max(minAz, minBz));

        const intersectionVol = xOverlap * yOverlap * zOverlap;
        if (intersectionVol <= 0) return 0;

        const volA = sizeA[0] * sizeA[1] * sizeA[2];
        const volB = sizeB[0] * sizeB[1] * sizeB[2];
        
        const unionVol = volA + volB - intersectionVol;
        
        return unionVol > 0 ? intersectionVol / unionVol : 0;
    }
}
