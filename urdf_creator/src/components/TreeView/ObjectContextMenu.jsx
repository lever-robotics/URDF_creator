import React from "react";

export function ObjectContextMenu({
    contextMenuPosition,
    selectedObject,
    stateFunctions,
}) {
    const { left, top } = contextMenuPosition;

    return (
        <div
            className="object-context-menu"
            // ref={objectContextMenu}
            style={{ left: left, top: top }}>
            <button
                onClick={() => {
                    stateFunctions.duplicateObject(selectedObject);
                }}
                className="duplicate-button">
                Duplicate
            </button>
            <button
                onClick={() => {
                    stateFunctions.deleteObject(selectedObject);
                }}
                className="delete-button">
                Delete
            </button>
        </div>
    );
}
