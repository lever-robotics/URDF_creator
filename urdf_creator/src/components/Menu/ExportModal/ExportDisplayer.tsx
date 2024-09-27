import React, { useState } from "react";
import ExportURDFPackage from "./ExportURDFPackage";
import ExportGLTF from "./ExportGLTF";
import ExportURDF from "./ExportURDF";
import "./exportDisplayer.css";
import Frame, { Frameish } from "../../../Models/Frame";
import { Scene } from "three";
import { JsxElement } from "typescript";
import ThreeScene from "../../ThreeDisplay/ThreeScene";

type Props = { onClose: () => void; projectTitle: string; threeScene: ThreeScene };

// the params originally were enclosed in {} braces but there was a type script compilation error that got fixed when I removed them
const ExportDisplayer: React.FC<Props> = ({ onClose, projectTitle, threeScene }) => {
    const [selectedIndex, setSelectedIndex] = useState(1);
    const scene = threeScene.scene;
    const rootFrame = threeScene.rootFrame;
    const [content, setContent] = useState(<ExportURDFPackage onClose={onClose} scene={scene} projectTitle={projectTitle} />);

    const exportOptions = [
        { label: "URDF", content: <ExportURDF onClose={onClose} scene={scene} projectTitle={projectTitle} /> },
        { label: "Robot Package", content: <ExportURDFPackage onClose={onClose} scene={scene} projectTitle={projectTitle} /> },
        { label: "GLTF", content: <ExportGLTF onClose={onClose} rootFrame={rootFrame} projectTitle={projectTitle} threeScene={threeScene} /> },
    ];

    return (
        <>
            <h2 className="title">Export Options</h2>
            <div className="import-displayer">
                <div className="import-menu-modal">
                    <ul className="menu-list">
                        {exportOptions.map((item, index) => (
                            <ExportOption key={index} index={index} item={item} setContent={setContent} selectedIndex={selectedIndex} setSelectedIndex={setSelectedIndex} />
                        ))}
                    </ul>
                </div>
                {content}
            </div>
        </>
    );
};

type Item = {
    label: string;
    content: JSX.Element;
}

type OptionProps = { item: Item; index: number; setContent: (e: JSX.Element) => void; selectedIndex: number; setSelectedIndex: (i: number) => void };

const ExportOption = ({ item, index, setContent, selectedIndex, setSelectedIndex }: OptionProps) => {
    const handleClick = () => {
        setContent(item.content);
        setSelectedIndex(index);
    };

    return (
        <li className={`menu-item ${selectedIndex === index ? "menu-selected" : ""}`} onClick={handleClick}>
            {item.label}
        </li>
    );
};

export default ExportDisplayer;
