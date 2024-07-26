import React, { useState, useEffect } from 'react';
import Inertia from '../../../Models/Inertia';

function InertiaParameters({ selectedObject, setInertia, setMass }) {
    const [mass, setMassTemp] = useState("");
    const [Ixx, setIxx] = useState('');
    const [Ixy, setIxy] = useState('');
    const [Ixz, setIxz] = useState('');
    const [Iyy, setIyy] = useState('');
    const [Iyz, setIyz] = useState('');
    const [Izz, setIzz] = useState('');

    useEffect(() => {
        if (selectedObject) {
            setMassTemp(selectedObject.inertia.mass || "");
            setIxx(selectedObject.inertia.ixx === 0 ? 0.0 : selectedObject.inertia.ixx.toFixed(3) || '');
            setIxy(selectedObject.inertia.ixy === 0 ? 0.0 : selectedObject.inertia.ixy.toFixed(3) || '');
            setIxz(selectedObject.inertia.ixz === 0 ? 0.0 : selectedObject.inertia.ixz.toFixed(3) || '');
            setIyy(selectedObject.inertia.iyy === 0 ? 0.0 : selectedObject.inertia.iyy.toFixed(3) || '');
            setIyz(selectedObject.inertia.iyz === 0 ? 0.0 : selectedObject.inertia.iyz.toFixed(3) || '');
            setIzz(selectedObject.inertia.izz === 0 ? 0.0 : selectedObject.inertia.izz.toFixed(3) || '');
        }
    }, [JSON.stringify(selectedObject.inertia)]);

    const handleInertiaChange = (prop, value) => {
        const newValue = parseFloat(value);
        if (isNaN(newValue)) return;
        const inertia = new Inertia(selectedObject.inertia.mass, parseFloat(Ixx), parseFloat(Iyy), parseFloat(Izz), parseFloat(Ixy), parseFloat(Ixz), parseFloat(Iyz));
        inertia.customInertia = true;
        setInertia(selectedObject, inertia);
    };

    const handleMassChange = (e) => {
        setMassTemp(e.target.value);
        setMass(selectedObject, parseFloat(e.target.value));
    };

    return (
        <div>
            <strong>Mass:</strong>
            <input type="number" value={mass} onChange={handleMassChange} />
            <span className="units">kg</span>
            <br />
            <br />
            <strong>Moment of Inertia:</strong>
            <ul>
                <li>
                    Ixx:
                    <input
                        type="number"
                        value={Ixx}
                        onChange={(e) => setIxx(e.target.value)}
                        onBlur={() => handleInertiaChange('Ixx', Ixx)}
                    />
                    <span className="units">km<span className="large-dot">·</span>m<sup>2</sup></span>
                </li>
                <li>
                    Ixy:
                    <input
                        type="number"
                        value={Ixy}
                        onChange={(e) => setIxy(e.target.value)}
                        onBlur={() => handleInertiaChange('Ixy', Ixy)}
                    />
                    <span className="units">km<span className="large-dot">·</span>m<sup>2</sup></span>
                </li>
                <li>
                    Ixz:
                    <input
                        type="number"
                        value={Ixz}
                        onChange={(e) => setIxz(e.target.value)}
                        onBlur={() => handleInertiaChange('Ixz', Ixz)}
                    />
                    <span className="units">km<span className="large-dot">·</span>m<sup>2</sup></span>
                </li>
                <li>
                    Iyy:
                    <input
                        type="number"
                        value={Iyy}
                        onChange={(e) => setIyy(e.target.value)}
                        onBlur={() => handleInertiaChange('Iyy', Iyy)}
                    />
                    <span className="units">km<span className="large-dot">·</span>m<sup>2</sup></span>
                </li>
                <li>
                    Iyz:
                    <input
                        type="number"
                        value={Iyz}
                        onChange={(e) => setIyz(e.target.value)}
                        onBlur={() => handleInertiaChange('Iyz', Iyz)}
                    />
                    <span className="units">km<span className="large-dot">·</span>m<sup>2</sup></span>
                </li>
                <li>
                    Izz:
                    <input
                        type="number"
                        value={Izz}
                        onChange={(e) => setIzz(e.target.value)}
                        onBlur={() => handleInertiaChange('Izz', Izz)}
                    />
                    <span className="units">km<span className="large-dot">·</span>m<sup>2</sup></span>
                </li>
            </ul>
        </div>
    );
}

export default InertiaParameters;
