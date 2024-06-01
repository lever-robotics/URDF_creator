export default function setUpSceneMouse(
    threeObjects,
    mountRef,
    mouseData,
    selectObject,
) {
    const { current: obj } = threeObjects;
    if (!mountRef.current || obj.initialized) return;

    function clickObject(event) {
        const rect = mountRef.current.getBoundingClientRect();
        const x = event.clientX - rect.left;
        const y = event.clientY - rect.top;

        obj.mouse.x = (x / rect.width) * 2 - 1;
        obj.mouse.y = -(y / rect.height) * 2 + 1;

        obj.raycaster.setFromCamera(obj.mouse, obj.camera);
        const intersects = obj.raycaster.intersectObjects(obj.scene.children);

        const shapes = intersects.filter(
            (collision) => collision.object.userData.shape
        );
        const meshes = intersects.filter(
            (collision) => collision.object.type === "Mesh"
        );

        if (shapes.length > 0) {
            const object = shapes[0].object;
            selectObject(object);
        } else if (meshes.length === 0) {
            selectObject(null);
        }
    }




    function onDoubleClick(event) {
        // Handle double click event if needed
    }

    function onClick(event) {
        clickObject(event);
    }

    function onMouseUp(event) {
        event.preventDefault();
        const clickTime = 300;
        const dragThreshold = 20;
        const endPos = [event.clientX, event.clientY];

        if (
            Math.sqrt(
                (endPos[0] - mouseData.current.startPos[0]) ** 2 +
                (endPos[1] - mouseData.current.startPos[1]) ** 2
            ) > dragThreshold
        ) {
            // Do nothing if dragged
        } else if (
            mouseData.current.currentDownTime -
            mouseData.current.previousUpTime <
            clickTime &&
            Date.now() - mouseData.current.currentDownTime < clickTime
        ) {
            onDoubleClick(event);
        } else if (Date.now() - mouseData.current.currentDownTime < clickTime) {
            onClick(event);
        }
        mouseData.current.previousUpTime = Date.now();
    }

    function onMouseDown(event) {
        event.preventDefault();
        mouseData.current.currentDownTime = Date.now();
        mouseData.current.startPos = [event.clientX, event.clientY];
    }

    mountRef.current.addEventListener("pointerdown", onMouseDown);
    mountRef.current.addEventListener("pointerup", onMouseUp);

    const callback = () => {
        if (mountRef.current) {
            mountRef.current.removeEventListener("pointerdown", onMouseDown);
            mountRef.current.removeEventListener("pointerup", onMouseUp);
        }
    };

    return callback;
}
