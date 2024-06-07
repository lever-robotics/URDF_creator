export default function GenerateLaunchFile(scene) {
    if (scene) {
        if (scene.children) {
            const children = scene.children.filter((child) => {
                return child instanceof urdfObject;
            });
            if (children.length > 0) {
                return children[0];
            }
        }
    }
}