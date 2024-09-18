import React from 'react';
import './onboarding.css';

/**
 * Page3 component that explains how to manipulate shapes in the URDF Creator tool.
 * 
 * @returns {JSX.Element} The rendered component.
 */
const Page3 = () => {
    return (
        <div className="page-container">
            <h2 className="section-header">Tools</h2>
            <div className="page3-container">
                <div className="tool-container">
                    <div className="text-section">
                        <h3 className="subsection-header-page3">Add Link</h3>
                        <p className="page3-description">
                            Add a link by first using the insert panel in bottem left. Manipulate the link by clicking on it or selecting it in the link tree.
                        </p>
                    </div>
                    <div className="graphic-container-page3">
                        <img src={'/statics/box.png'} alt="Add Cube" className="graphic-page3"/>
                    </div>
                </div>

                <div className="tool-container">
                    <div className="text-section">
                        <h3 className="subsection-header-page3">Translation</h3>
                        <p className="page3-description">
                            Move the link accordingly by dragging it on each of the three axis. Give exact position by setting position in link parameters.
                        </p>
                    </div>
                    <div className="graphic-container-page3">
                        <img src={'/statics/translate.png'} alt="Translation" className="graphic-page3"/>
                    </div>
                </div>

                <div className="tool-container">
                    <div className="text-section">
                        <h3 className="subsection-header-page3">Scale</h3>
                        <p className="page3-description">
                            To give specific scaling, positioning, or rotation of a link, use the parameters in the Object Parameters section.
                        </p>
                    </div>
                    <div className="graphic-container-page3">
                        <img src={'/statics/scale.png'} alt="Scale" className="graphic-page3"/>
                    </div>
                </div>

                <div className="tool-container">
                    <div className="text-section">
                        <h3 className="subsection-header-page3">Link Parameters</h3>
                        <p className="page3-description"> 
                            Link Parameters is all the meta data you can specify about a link including is size, position, and rotation.
                            Aswell as its color, name, inertia properties and a visual stl or dae file.
                        </p>
                    </div>
                    <div className="graphic-container-page3">
                        <img src={'/statics/params.png'} alt="Object Parameters" className="graphic-page3"/>
                    </div>
                </div>

                <div className="tool-container">
                    <div className="text-section">
                        <h3 className="subsection-header-page3">Duplicate</h3>
                        <p className="page3-description">
                        You can use the duplicate function by right-clicking on the link in the Link Tree and selecting duplicate. This is also a way to delete a link.
                        </p>
                    </div>
                    <div className="graphic-container-page3">
                        <img src={'/statics/duplicate.png'} alt="Duplicate" className="graphic-page3"/>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default Page3;
