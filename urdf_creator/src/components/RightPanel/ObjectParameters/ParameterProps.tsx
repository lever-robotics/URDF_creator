import { Frameish } from "../../../Models/Frame";
import ThreeScene from "../../ThreeDisplay/ThreeScene";

// Parameter Props for parameters at the Frame level
type ParameterProps = {
    selectedObject?: Frameish,
    threeScene: ThreeScene,
}

export default ParameterProps;