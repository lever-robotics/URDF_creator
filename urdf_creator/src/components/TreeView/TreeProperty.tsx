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

    return (
        <button
            key={property.id}
            className={styles.treeProperty}
            onContextMenu={onContextMenu}
            draggable={true}
            onClick={onClick}
            onDragEnd={onDragEnd}>
            {property.name}
        </button>
    );
}
