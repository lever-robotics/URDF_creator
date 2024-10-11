import type Frame from "../../../Models/Frame";
import { Frameish } from "../../../Models/Frame";
import type Inertia from "../../../Models/Inertia";
import type Collision from "../../../Models/VisualCollision";
import type { Visual } from "../../../Models/VisualCollision";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";

//* Used for parameters that can be passed other selected objects such as [Visual, Collision, Inertia, Joint]
type ItemParameterProps = {
    selectedObject?: Frame | Visual | Collision | Inertia;
    selectedItem?: Visual | Collision | Inertia;
    threeScene: ThreeScene;
};

export default ItemParameterProps;
