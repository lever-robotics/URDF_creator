import { ScenetoSDF } from "./ScenetoSDF";
import { ScenetoXML } from "./ScenetoXML";
import { ScenetoJSON } from "./ScenetoJSON";
import * as THREE from "three";

/**
 * Returns the scene in text form in the specified format
 * @param {string} format 
 * @param {Scene} scene 
 * @param {string} projectTitle 
 * @returns Text representation of the scene object in desired format
 */
export default function ScenetoText(format: string, scene: THREE.Scene, projectTitle: string) {
    switch(format){
        case 'URDF': 
            return ScenetoXML(scene, projectTitle);
        case "XML":
            return ScenetoXML(scene, projectTitle);
        case 'SDF':
            return ScenetoSDF(scene, projectTitle);
        case "XACRO":
            return "Xacro not currently supported. This format is for programmatically writing urdf files.";
        case 'JSON':
            return ScenetoJSON(scene, projectTitle);
        default:
            return "Error in ScenetoText";
    }
}