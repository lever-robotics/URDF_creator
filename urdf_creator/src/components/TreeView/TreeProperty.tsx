import Frame, { Frameish } from "../../Models/Frame";
import ThreeScene from "../ThreeDisplay/ThreeScene";
import { ContextMenu } from "./LinkTree";
import styles from "./TreeFrame.module.css";

// TODO all Framish types need to become the superset selectedObject type
type Props = {
    property: any;
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
        if (selectedProperty instanceof Frame)return false;
        if (selectedProperty.id === property.id) return true;
        return false;
    };

    //TODO
    const onClick = () => {
        threeScene.selectObject(property);
    };

    //TODO
    const onDragEnd = (e: React.MouseEvent) => {
        // if (hoveredFrame) {
        //     hoveredFrame.attachChild(property);
        //     threeScene.selectObject(hoveredFrame); // selectItem
        //     setHoveredFrame(null);
        // }
    };

    const onContextMenu = (e: React.MouseEvent) => {
        handleContextMenu(e, property);
    };

    const selectedStyle = isSelected() ? {borderColor: "#646cff"}: {};

    return (
        <button
            key={property.id}
            className={styles.treeProperty}
            style={selectedStyle}
            onContextMenu={onContextMenu}
            draggable={true}
            onClick={onClick}
            onDragEnd={onDragEnd}>
            {property.name}
        </button>
    );
}
