import * as THREE from "three";

export const ScenetoJSON = (scene, filename) => {
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

export async function JSONtoScene(jsonstring){
    const json = JSON.parse(jsonstring);
    console.log(json);
    const loader = new THREE.ObjectLoader();
    const scene = loader.parse(jsonstring, () => {
        
    });
    return scene;
};