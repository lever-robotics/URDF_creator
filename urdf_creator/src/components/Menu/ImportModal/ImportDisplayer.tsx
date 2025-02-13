import type React from "react";
import { useRef, useState } from "react";
import GLTFImport from "./GLTFImport";
import GltfFilesGrid from "./ImportSensor";
import STLImport from "./STLImport";
import "./importDisplayer.css";
import type { Object3D } from "three";
import SolidWorksImport from "./SolidWorksImport";

type Props = {
    handleSensorClick: (path: string) => void;
    onImportClose: () => void;
    loadScene: (object: Object3D) => void;
};

const ImportDisplayer: React.FC<Props> = ({
    handleSensorClick,
    onImportClose,
    loadScene,
}) => {
    const [content, setContent] = useState(
        <GLTFImport onClose={onImportClose} loadScene={loadScene} />,
    );
    const [selectedIndex, setSelectedIndex] = useState(2);

    const importOptions = [
        { label: "STL", content: <STLImport onClose={onImportClose} /> },
        {
            label: "SolidWorks",
            content: <SolidWorksImport onClose={onImportClose} />,
        },
        {
            label: "Local Project",
            content: (
                <GLTFImport onClose={onImportClose} loadScene={loadScene} />
            ),
        },
        {
            label: "Robot Sensor",
            content: <GltfFilesGrid handleSensorClick={handleSensorClick} />,
        },
        // { label: "Sensors", content: },
        // { label: "Link", content: },
        // { label: "URDF", content: },
    ];

    return (
        <>
            <h2 className="title">Import Options</h2>
            <div className="import-displayer">
                <div className="import-menu-modal">
                    <ul className="menu-list">
                        {importOptions.map((item, index) => (
                            <ImportOption
                                key={item.label}
                                index={index}
                                item={item}
                                setContent={setContent}
                                selectedIndex={selectedIndex}
                                setSelectedIndex={setSelectedIndex}
                            />
                        ))}
                    </ul>
                </div>
                {content}
            </div>
        </>
    );
};

type ImportProps = {
    item: {
        label: string;
        content: JSX.Element;
    };
    index: number;
    setContent: (content: JSX.Element) => void;
    selectedIndex: number;
    setSelectedIndex: (index: number) => void;
};

const ImportOption: React.FC<ImportProps> = ({
    item,
    index,
    setContent,
    selectedIndex,
    setSelectedIndex,
}) => {
    const handleClick = () => {
        setContent(item.content);
        setSelectedIndex(index);
    };

    return (
        <li
            className={`menu-item ${selectedIndex === index ? "menu-selected" : ""}`}
            onClick={handleClick}
        >
            {item.label}
        </li>
    );
};

export default ImportDisplayer;
