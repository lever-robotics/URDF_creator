import React from 'react';
import './STLImport.css';

const STLImport = () => {
    return (
        <div className='stl-container'>
            <div className="header">
                <div className="stl-import-title">
                    <h3>Import a STL File</h3>
                </div>
                <div className="import-box">
                    <button className='stl-import-button'>Import STL</button>
                </div>
            </div>
            <div className="description-container">
                <h3>STL Description</h3>
                <p>An STL (Stereolithography) file is a widely used file format for 3D printing and computer-aided design (CAD). 
                   It represents the surface geometry of a 3D object using a series of triangles. 
                   When you import an STL file, you are bringing in a 3D model that can be manipulated or used for various applications such as 3D printing, simulation, or analysis.</p>
                <p>Click the button above to import an STL file. Once imported, you can view, edit, or utilize the 3D model in your project.</p>
            </div>
            <div className="image-container">
                <div className="image-placeholder">Image</div>
                <div className="image-description">Example of an STL File Representation</div>
            </div>
        </div>
    );
}

export default STLImport;