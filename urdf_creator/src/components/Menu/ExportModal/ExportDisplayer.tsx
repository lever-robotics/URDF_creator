import React, { useState } from "react";
import ExportURDFPackage from "./ExportURDFPackage";
import ExportGLTF from "./ExportGLTF";
import ExportURDF from "./ExportURDF";
import "./exportDisplayer.css";
import { StateFunctionsType } from "../../SceneState";
import Frame, { Frameish } from "../../../Models/Frame";
import { Scene } from "three";
import { JsxElement } from "typescript";

type Props = { onClose: () => void; getRootFrame: () => Frameish; projectTitle: string; getScene: () => Scene; stateFunctions: StateFunctionsType };

// the params originally were enclosed in {} braces but there was a type script compilation error that got fixed when I removed them
const ExportDisplayer: React.FC<Props> = ({ onClose, getRootFrame, projectTitle, getScene, stateFunctions }) => {
    const [selectedIndex, setSelectedIndex] = useState(1);
    const [content, setContent] = useState(<ExportURDFPackage onClose={onClose} getScene={getScene} projectTitle={projectTitle} />);

    const exportOptions = [
        { label: "URDF", content: <ExportURDF onClose={onClose} getScene={getScene} projectTitle={projectTitle} /> },
        { label: "Robot Package", content: <ExportURDFPackage onClose={onClose} getScene={getScene} projectTitle={projectTitle} /> },
        { label: "GLTF", content: <ExportGLTF onClose={onClose} getRootFrame={getRootFrame} projectTitle={projectTitle} stateFunctions={stateFunctions} /> },
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
