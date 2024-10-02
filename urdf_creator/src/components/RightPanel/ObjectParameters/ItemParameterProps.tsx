import { Frameish } from "../../../Models/Frame";
import ThreeScene from "../../ThreeDisplay/ThreeScene";
import Collision, {Visual} from "../../../Models/VisualCollision";
import Inertia from "../../../Models/Inertia";


//* Used for parameters that can be passed other selected objects such as [Visual, Collision, Inertia, Joint]
type ItemParameterProps = {
    selectedObject?: Frameish,
    selectedItem?: Frameish | Visual | Collision | Inertia,
    threeScene: ThreeScene,
}

export default ItemParameterProps;