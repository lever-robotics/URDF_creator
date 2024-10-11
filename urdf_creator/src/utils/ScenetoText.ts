import * as THREE from "three";
import type ThreeScene from "../components/ThreeDisplay/ThreeScene";
import { ScenetoJSON } from "./ScenetoJSON";
import { ScenetoSDF } from "./ScenetoSDF";
import { ScenetoXML } from "./ScenetoXML";

/**
 * Returns the scene in text form in the specified format
 * @param {string} format
 * @param {Scene} scene
 * @param {string} projectTitle
 * @returns Text representation of the scene object in desired format
 */
export default function ScenetoText(
    format: string,
    scene: ThreeScene,
    projectTitle: string,
) {
    switch (format) {
        case "URDF":
            return ScenetoXML(scene, projectTitle);
        case "XML":
            return ScenetoXML(scene, projectTitle);
        case "SDF":
            return ScenetoSDF(scene, projectTitle);
        case "XACRO":
            return "Xacro not currently supported. This format is for programmatically writing urdf files.";
        default:
            return "Error in ScenetoText";
    }
}
