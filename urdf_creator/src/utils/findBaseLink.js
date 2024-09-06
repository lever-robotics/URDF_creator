import Frame from "../Models/Frame";

export default function findBaseLink(scene) {
    if (scene) {
        if (scene.children) {
            const children = scene.children.filter((child) => {
                return child instanceof Frame;
            });
            if (children.length > 0) {
                return children[0];
            }
        }
    }
}
