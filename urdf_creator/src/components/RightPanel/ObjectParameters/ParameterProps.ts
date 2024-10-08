import Frame, { Frameish } from "../../../Models/Frame";
import Inertia from "../../../Models/Inertia";
import { Collision, Visual } from "../../../Models/VisualCollision";
import ThreeScene from "../../ThreeDisplay/ThreeScene";

// Parameter Props for parameters at the Frame level
type ParameterProps = {
    selectedObject?: Frame | Visual | Collision | Inertia,
    threeScene: ThreeScene,
}

export type ParameterValue = string | number | undefined;

export default ParameterProps;