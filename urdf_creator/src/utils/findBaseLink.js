export default function findBaseLink(scene) {
    if (scene) {
        if (scene.children) {
            const children = scene.children.filter((child) => {
                return child.sceneObject;
            });
            if (children.length > 0) {
                return children[0];
            }
        }
    }
}