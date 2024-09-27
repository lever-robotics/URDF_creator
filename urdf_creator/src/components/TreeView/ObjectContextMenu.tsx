import React from "react";
import Frame, { Frameish } from "../../Models/Frame";
import ThreeScene from "../ThreeDisplay/ThreeScene";

type ContextProps = {
    contextMenuPosition: {left: number, top: number},
    selectedObject: Frameish,
    threeScene: ThreeScene,
}

export function ObjectContextMenu({
    contextMenuPosition,
    selectedObject,
    threeScene,
}: ContextProps) {
    const { left, top } = contextMenuPosition;
    return (
        <div
            className="object-context-menu"
            // ref={objectContextMenu}
            style={{ left: left, top: top }}>
            <button
                onClick={() => {
                    threeScene?.duplicateObject(selectedObject!);
                }}
                className="duplicate-button">
                Duplicate
            </button>
            <button
                onClick={() => {
                    threeScene?.deleteObject(selectedObject!);
                }}
                className="delete-button">
                Delete
            </button>
        </div>
    );
}
