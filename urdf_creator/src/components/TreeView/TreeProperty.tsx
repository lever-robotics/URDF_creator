import Frame, { type Frameish } from "../../Models/Frame";
import VisualCollision, {
    type Collision,
    type Visual,
} from "../../Models/VisualCollision";
import type ThreeScene from "../ThreeDisplay/ThreeScene";
import type { ContextMenu } from "./LinkTree";
import styles from "./TreeFrame.module.css";

// TODO all Framish types need to become the superset selectedObject type
type Props = {
    property: Visual | Collision;
    handleContextMenu: (e: React.MouseEvent, treeObject: ContextMenu) => void;
    threeScene: ThreeScene;
    hoveredFrame: Frameish;
    setHoveredFrame: (f: Frameish) => void;
};

export default function TreeProperty(props: Props) {
    // Destructure props
    const {
        property,
        handleContextMenu,
        threeScene,
        hoveredFrame,
        setHoveredFrame,
    } = props;

    const isSelected = () => {
        const selectedProperty = threeScene.selectedObject;
        if (!selectedProperty) return false;
        if (selectedProperty instanceof Frame) return false;
        if (selectedProperty.id === property.id) return true;
        return false;
    };

    const onClick = () => {
        if (!(property instanceof Frame)) threeScene.selectObject(property);
    };

    const onDragEnd = (e: React.MouseEvent) => {
        if (hoveredFrame) {
            property.frame.removeProperty(property);
            hoveredFrame.addProperty(property);
            threeScene.selectObject(hoveredFrame);
            setHoveredFrame(null);
        }
    };

    const onContextMenu = (e: React.MouseEvent) => {
        handleContextMenu(e, property);
    };

    const selectedStyle = isSelected() ? { borderColor: "#646cff" } : {};

    return (
        <button
            key={property.id}
            className={styles.treeProperty}
            style={selectedStyle}
            onContextMenu={onContextMenu}
            draggable={true}
            onClick={onClick}
            onDragEnd={onDragEnd}
            type="button"
        >
            {property.name}
        </button>
    );
}
