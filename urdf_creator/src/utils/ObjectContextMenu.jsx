import React from "react";

export function ObjectContextMenu({ objectContextMenu, objectContextMenuPosition, selectedObject, updateTree, setSelectedObject, transformControls }) {

	const { left, top } = objectContextMenuPosition;


	const copyOnBeforeRender = (object, clone) => {
		if (!object || !clone) return;

		// make the clone onBeforeRender be the same as the original
		clone.onBeforeRender = object.onBeforeRender
		for (let i = 0; i < object.children.length; i++) {
			copyOnBeforeRender(object.children[i], clone.children[i])
		}
	}

	const duplicateObject = () => {
		const clone = selectedObject.clone(true);

		//This copies the onBeforeRender callback into the clone
		copyOnBeforeRender(selectedObject, clone)

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

