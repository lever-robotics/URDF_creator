import urdfObject from "../Models/urdfObject";

export default function findBaseLink(scene) {
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