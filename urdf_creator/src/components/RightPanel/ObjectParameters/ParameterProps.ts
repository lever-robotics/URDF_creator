import type Frame from "../../../Models/Frame";
import { Frameish } from "../../../Models/Frame";
import type Inertia from "../../../Models/Inertia";
import type { Collision, Visual } from "../../../Models/VisualCollision";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";

// Parameter Props for parameters at the Frame level
type ParameterProps = {
    selectedObject: Frame | Visual | Collision | Inertia;
    threeScene: ThreeScene;
};

export type ParameterValue = string | number | undefined;

export default ParameterProps;
