import React from "react";

export function ObjectContextMenu({
    objectContextMenu,
    objectContextMenuPosition,
    duplicateObject,
    deleteObject,
    selectedObject,
}) {
    const { left, top } = objectContextMenuPosition;

    return (
        <div
            className="object-context-menu"
            ref={objectContextMenu}
            style={{ left: left, top: top }}>
            <button
                onClick={() => {
                    duplicateObject(selectedObject);
                }}
                className="duplicate-button">
                Duplicate
            </button>
            <button
                onClick={() => {
                    deleteObject(selectedObject);
                }}
                className="delete-button">
                Delete
            </button>
        </div>
    );
}
