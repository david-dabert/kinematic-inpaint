/**
 * Kinematic Occlusion Inpainting Engine
 *
 * Implements rigid skeletal segment length preservation and 3D geometric
 * trilateration to solve severe landmark occlusions in strength training
 * (e.g., 45 cm Olympic bumper plates crossing knee/ankle trajectories during squats,
 * or machine chest pads/shields occluding torso/hip kinematics).
 *
 * Biomechanical Principles:
 * 1. Rigid Link Invariance: Human adult long bones (femur, tibia, humerus, radius)
 *    and spinal trunk height are strictly rigid bodies. Their 3D Euclidean lengths
 *    are invariant across dynamic flexion/extension cycles.
 * 2. Sphere-Sphere Trilateration: An occluded intermediate joint (e.g. Knee)
 *    between two visible anchors (Hip and Ankle) is mathematically constrained
 *    to the intersection circle of two 3D spheres centered at the anchors with radii
 *    equal to the calibrated segment lengths (Winter 2009, Dempster 1955).
 * 3. Kalman Velocity Continuation: A constant-velocity 3D kinematic Kalman filter
 *    tracks joint momentum. When landmark visibility drops below threshold,
 *    velocity continuation predicts trajectory momentum, which is then snapped
 *    onto the exact sphere intersection manifold.
 *
 * References:
 * - Winter, D. A. (2009). Biomechanics and Motor Control of Human Movement.
 * - Dempster, W. T. (1955). Space requirements of the seated operator.
 * - Chaffin, D. B., & Andersson, G. (1991). Occupational Biomechanics.
 */
export declare const LANDMARKS: {
    readonly NOSE: 0;
    readonly LEFT_SHOULDER: 11;
    readonly RIGHT_SHOULDER: 12;
    readonly LEFT_ELBOW: 13;
    readonly RIGHT_ELBOW: 14;
    readonly LEFT_WRIST: 15;
    readonly RIGHT_WRIST: 16;
    readonly LEFT_HIP: 23;
    readonly RIGHT_HIP: 24;
    readonly LEFT_KNEE: 25;
    readonly RIGHT_KNEE: 26;
    readonly LEFT_ANKLE: 27;
    readonly RIGHT_ANKLE: 28;
    readonly LEFT_HEEL: 29;
    readonly RIGHT_HEEL: 30;
    readonly LEFT_FOOT_INDEX: 31;
    readonly RIGHT_FOOT_INDEX: 32;
};
export interface Landmark3D {
    x: number;
    y: number;
    z?: number;
    visibility?: number;
}
export interface Point3D {
    x: number;
    y: number;
    z: number;
}
export interface SkeletalInvariants {
    leftFemur: number;
    rightFemur: number;
    leftTibia: number;
    rightTibia: number;
    leftTorso: number;
    rightTorso: number;
    leftHumerus?: number;
    rightHumerus?: number;
    leftForearm?: number;
    rightForearm?: number;
}
export interface InpaintedJointInfo {
    index: number;
    name: string;
    isOccluded: boolean;
    isInpainted: boolean;
    confidence: number;
    originalVisibility: number;
    originalPoint: Point3D;
    projectedPoint: Point3D;
    method: 'sphere_kalman_continuation' | 'kalman_continuation' | 'unmodified';
}
export interface InpaintedFrameResult {
    landmarks: Landmark3D[];
    inpaintedJoints: InpaintedJointInfo[];
    invariantsCompliant: boolean;
    maxInvariantDeviation: number;
}
export interface KinematicOcclusionInpainterConfig {
    /** Visibility below which inpainting triggers (default: 0.50) */
    visibilityThreshold?: number;
    /** Minimum anchor visibility required for geometric trilateration (default: 0.35) */
    minAnchorVisibility?: number;
    /** Process noise acceleration standard deviation for Kalman continuation (default: 2.0 m/s^2) */
    processNoiseAcc?: number;
    /** Measurement noise for raw computer vision observations (default: 0.02) */
    measurementNoise?: number;
    /** Virtual observation noise when updating with inpainted coordinates (default: 0.03) */
    inpaintedMeasurementNoise?: number;
    /** Calibrated skeletal segment invariants */
    invariants?: SkeletalInvariants;
}
export interface SphereIntersectionResult {
    point: Point3D;
    circleCenter: Point3D;
    circleRadius: number;
    isLockout: boolean;
    isDegenerate: boolean;
}
/**
 * 3D Euclidean distance between two landmarks.
 */
export declare function computeDistance3D(a: Landmark3D, b: Landmark3D): number;
/**
 * 1D Constant-Velocity Kinematic Kalman Filter for decoupled Cartesian axes.
 * Implements Discrete Continuous White Noise Acceleration (CWNA) model.
 */
export declare class AxisKalmanFilter {
    sigmaA: number;
    sigmaM: number;
    p: number;
    v: number;
    p00: number;
    p01: number;
    p10: number;
    p11: number;
    constructor(sigmaA?: number, sigmaM?: number);
    reset(initialP?: number, initialV?: number): void;
    predict(dt: number): number;
    update(meas: number, customSigmaM?: number): void;
}
/**
 * 3D Kinematic Kalman Tracker for tracking joint trajectory and continuing velocity.
 */
export declare class JointKalmanTracker3D {
    private xFilter;
    private yFilter;
    private zFilter;
    private lastTimestampMs;
    private isInitialized;
    constructor(processNoiseAcc?: number, measurementNoisePos?: number);
    initialized(): boolean;
    init(p: Point3D, timestampMs: number, initialVelocity?: Point3D): void;
    reset(): void;
    predict(timestampMs: number): Point3D;
    update(observed: Point3D): void;
    updateInpainted(projected: Point3D, customNoise?: number): void;
    getState(): {
        position: Point3D;
        velocity: Point3D;
    };
}
/**
 * Geometric Sphere-Sphere Trilateration Solver.
 * Computes the exact 3D intersection manifold of two rigid spheres (centers C1, C2
 * with radii r1, r2) and projects the continuation point onto the intersection circle.
 *
 * Guarantees ||P - C1|| == r1 and ||P - C2|| == r2 to machine precision.
 */
export declare function intersectAdjacentSpheres(center1: Point3D, radius1: number, center2: Point3D, radius2: number, continuationPoint: Point3D, preferredFlexionNormal?: Point3D): SphereIntersectionResult;
/**
 * Calibrate rigid skeletal invariants from unoccluded standing setup frames.
 */
export declare function calibrateSkeletalInvariants(frames: Landmark3D[][], minVisibility?: number): SkeletalInvariants | null;
/**
 * Verify skeletal invariants on a landmark frame.
 * Checks that bone segment lengths conform to calibrated invariants within specified tolerance.
 */
export declare function verifySkeletalInvariants(landmarks: Landmark3D[], invariants: SkeletalInvariants, tolerance?: number): {
    compliant: boolean;
    maxDeviation: number;
    deviations: {
        leftFemur?: number;
        rightFemur?: number;
        leftTibia?: number;
        rightTibia?: number;
        leftTorso?: number;
        rightTorso?: number;
    };
};
/**
 * Kinematic Occlusion Inpainter
 * Real-time edge processor enforcing rigid segment length preservation and Kalman continuation.
 */
export declare class KinematicOcclusionInpainter {
    private config;
    private trackers;
    private invariants;
    constructor(config?: KinematicOcclusionInpainterConfig);
    setInvariants(invariants: SkeletalInvariants): void;
    getInvariants(): SkeletalInvariants | null;
    calibrateFromStanding(standingFrames: Landmark3D[][]): SkeletalInvariants | null;
    reset(): void;
    /**
     * Process a single landmark frame with kinematic occlusion inpainting.
     */
    processFrame(rawLandmarks: Landmark3D[], timestampMs: number): InpaintedFrameResult;
    /**
     * Process a full video sequence of landmark frames.
     */
    processSequence(frames: Landmark3D[][], timestampsMs: number[]): InpaintedFrameResult[];
}
//# sourceMappingURL=index.d.ts.map