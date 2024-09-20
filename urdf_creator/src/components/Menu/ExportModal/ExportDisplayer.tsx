import React, { useState } from "react";
import ExportURDFPackage from "./ExportURDFPackage";
import ExportGLTF from "./ExportGLTF";
import ExportURDF from "./ExportURDF";
import "./exportDisplayer.css";
import { StateFunctionsType } from "../../SceneState";

type Props = { onClose: () => void; getRootFrame: any; projectTitle: any; getScene: any; stateFunctions: StateFunctionsType };

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

type OptionProps = { item: any; index: any; setContent: any; selectedIndex: any; setSelectedIndex: any };

const ExportOption: React.FC<OptionProps> = ({ item, index, setContent, selectedIndex, setSelectedIndex }) => {
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
