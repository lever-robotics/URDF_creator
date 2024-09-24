import { StateFunctionsType } from "../../SceneState";
import Frame, { Frameish } from "../../../Models/Frame";

type ParameterProps = {
    selectedObject?: Frameish,
    stateFunctions: StateFunctionsType,
}

export default ParameterProps;