import React from "react";
import Frame from "../../Models/Frame";
import { StateFunctionsType } from "../SceneState";

type ContextProps = {
    contextMenuPosition: {left: number, top: number},
    selectedObject: Frame | null | undefined,
    stateFunctions: StateFunctionsType
}

export function ObjectContextMenu({
    contextMenuPosition,
    selectedObject,
    stateFunctions,
}: ContextProps) {
    const { left, top } = contextMenuPosition;

    return (
        <div
            className="object-context-menu"
            // ref={objectContextMenu}
            style={{ left: left, top: top }}>
            <button
                onClick={() => {
                    stateFunctions.duplicateObject(selectedObject!);
                }}
                className="duplicate-button">
                Duplicate
            </button>
            <button
                onClick={() => {
                    stateFunctions.deleteObject(selectedObject!);
                }}
                className="delete-button">
                Delete
            </button>
        </div>
    );
}
