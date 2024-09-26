import { StateFunctionsType } from "../../SceneState";
import Frame, { Frameish } from "../../../Models/Frame";
import ThreeScene from "../../ThreeDisplay/ThreeSceneObject";

type ParameterProps = {
    selectedObject?: Frameish,
    threeScene: ThreeScene,
}

export default ParameterProps;