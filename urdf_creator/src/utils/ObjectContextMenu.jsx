import React from "react";

export function ObjectContextMenu({ objectContextMenu, objectContextMenuPosition, selectedObject, updateTree, setSelectedObject, transformControls }) {

	const { left, top } = objectContextMenuPosition;

	const duplicateObject = () => {
		const clone = selectedObject.clone(true);
		selectedObject.parent.add(clone);
		updateTree()
		setSelectedObject(null)
	}

	const deleteObject = () => {
		transformControls.detach()
		selectedObject.removeFromParent()
		updateTree()
		setSelectedObject(null)
	}

	return (<div className="object-context-menu" ref={objectContextMenu} style={{ left: left, top: top }}>
		<button onClick={duplicateObject} className="duplicate-button">Duplicate</button>
		<button onClick={deleteObject} className="delete-button">Delete</button>
	</div>);
}
