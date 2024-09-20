import * as THREE from "three";

export const ScenetoJSON = (scene: THREE.Scene, filename: string) => {
    const jsonObject = scene.toJSON();
    const blob = new Blob([JSON.stringify(jsonObject)], {
        type: 'application/json',
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${filename}.json`;
    a.click();
    URL.revokeObjectURL(url);
};

export async function JSONtoScene(jsonstring: string) {
    const json = JSON.parse(jsonstring);
    const loader = new THREE.ObjectLoader();
    const scene = loader.parse(jsonstring, () => {

    });
    return scene;
};