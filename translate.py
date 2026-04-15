import os
import re
from deep_translator import GoogleTranslator
def main():
    translator = GoogleTranslator(source='en', target='zh-CN')
    
    files_to_translate = [
        'src/blimp_dynamics.m',
        'src/blimp_kinematics.m',
        'src/blimp_params.m',
        'src/esn_observer.m',
        'src/tablf_controller.m',
        'tests/test_closed_loop_control.m'
    ]
    
    for filepath in files_to_translate:
        if not os.path.exists(filepath):
            print(f"File not found: {filepath}")
            continue
            
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # update save() in test_closed_loop_control.m
        if 'test_closed_loop_control.m' in filepath:
            old_save = "save(fullfile(data_dir, 'simulation_data.mat'), 't_span', 'x1', 'x2', 'hat_x1', 'hat_x2', 'tau_c', 'x1_d', 'hat_tau_D');"
            new_save = "save(fullfile(data_dir, 'simulation_data.mat'), 't_span', 'x1', 'x2', 'hat_x1', 'hat_x2', 'tau_c', 'x1_d', 'hat_tau_D', 'kh', 'kl');"
            content = content.replace(old_save, new_save)
        
        lines = content.split('\n')
        new_lines = []
        for line in lines:
            match = re.search(r'^(.*?)(%+)(.*)$', line)
            if match:
                code = match.group(1)
                pct = match.group(2)
                comment = match.group(3)
                
                if re.search(r'[a-zA-Z]', comment):
                    try:
                        translated = translator.translate(comment.strip())
                        line = f"{code}{pct} {translated}"
                    except Exception as e:
                        pass
            new_lines.append(line)
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write('\n'.join(new_lines))
            
        print(f"Processed: {filepath}")

if __name__ == "__main__":
    main()
