import { StateFunctionsType } from "../../SceneState";
import Frame from "../../../Models/Frame";

type ParameterProps = {
    selectedObject?: Frame | null,
    stateFunctions: StateFunctionsType,
}

export default ParameterProps;